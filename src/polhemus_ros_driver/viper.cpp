/*

 Copyright (C) 2022-2023 Shadow Robot Company Ltd <software@shadowrobot.com>

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA

*/


#include <polhemus_ros_driver/viper.hpp>
#include <ros/console.h>

#include <string>

#ifdef DEBUG
#include <stdio.h>
#define warn(as...) \
{ \
  fprintf(stderr, "%s:%d: ", __FILE__, __LINE__); \
  fprintf(stderr, as); \
}
#else
#define warn(as...)
#endif

#define VALIDATE_CONTEXT(pctx, ctx) \
{ \
  pctx = reinterpret_cast<CVPcontext*>(ctx); \
  if (!(CVPcontext::findPctx(pctx))) return E_VPERR_INVALID_CONTEXT; \
}

Viper::Viper(std::string name, uint16_t rx_buffer_size, uint16_t tx_buffer_size,
             uint8_t sensors_right_glove, uint8_t sensors_left_glove)
    : Polhemus(name, rx_buffer_size, tx_buffer_size, sensors_right_glove, sensors_left_glove)
{
}

Viper::~Viper(void) {}

void Viper::device_init()
{
  source_select_service = nh->advertiseService("setting_source", &Viper::src_select_srv, this);
}

int Viper::device_reset(void)
{
  int return_value = device_data_mode(DATA_RESET);
  device_clear_input();
  return return_value;
}

int Viper::device_data_mode(data_mode_e mode)
{
  viper_cmds_e cmd_type;
  viper_cmd_actions_e action;
  CVPcmd viper_command;
  switch (mode)
  {
    case DATA_CONTINUOUS:
      cmd_type = CMD_CONTINUOUS_PNO;
      action = CMD_ACTION_SET;
      break;
    case DATA_SINGLE:
      cmd_type = CMD_SINGLE_PNO;
      action = CMD_ACTION_GET;
      break;
    case DATA_RESET:
      cmd_type = CMD_CONTINUOUS_PNO;
      action = CMD_ACTION_RESET;
      break;
    default:
      return RETURN_ERROR;
  }
  viper_command.Fill(cmd_type, action);
  viper_command.Prepare(g_txbuf, g_ntxcount);
  int nBytes = g_ntxcount;
  uint8_t *pbuf = g_txbuf;
  int return_value = device_send(pbuf, nBytes);
  if (return_value != RETURN_ERROR && action != CMD_ACTION_RESET)
  {
    return_value = receive_data_frame(cmd_type);
  }
  return return_value;
}

int Viper::receive_data_frame(viper_cmds_e cmd_type)
{
  int return_value = RETURN_ERROR;
  g_nrxcount = VIPER_RX_BUF_SIZE;
  int attempts = 3;
  for (int attempt = 0; attempt < attempts; attempt++)
  {
    return_value = device_read(g_rxbuf, g_nrxcount, true);
    if (return_value == 0)
    {
      CFrameInfo frame_info(g_rxbuf, g_nrxcount);
      if ((frame_info.cmd() == -1) || (frame_info.action() == -1))
      {
        if (attempt < attempts - 1)
        {
          ROS_WARN("[POLHEMUS] Message reply is not valid, retrying...");
          ROS_DEBUG("[POLHEMUS] Attempt %d of %d", attempt + 1, attempts);
          ROS_DEBUG("reply action: %d", frame_info.action());
          ROS_DEBUG("cmd sent: %d", cmd_type);
          continue;
        }
        else
        {
          ROS_ERROR("[POLHEMUS] Message reply is not valid after %d attempts, aborting...", attempts);
          ROS_ERROR("reply action: %d", frame_info.action());
          ROS_ERROR("cmd sent: %d", cmd_type);
          return_value = RETURN_ERROR;
          break;
        }
      }
      else if ((frame_info.cmd() != cmd_type) || !(frame_info.IsAck()))
      {
        ROS_ERROR("[POLHEMUS] Error in message reply...");
        ROS_ERROR("reply cmd: %d", frame_info.cmd());
        ROS_ERROR("reply action: %d", frame_info.action());
        ROS_ERROR("cmd sent: %d", cmd_type);
        return_value = RETURN_ERROR;
        break;
      }
      else
      {
        ROS_DEBUG("[POLHEMUS] Got expected message reply at attempt %d", attempt);
        break;
      }
    }
  }
  return return_value;
}

int Viper::receive_pno_data_frame(void)
{
  int return_value = RETURN_ERROR;
  g_nrxcount = VIPER_RX_BUF_SIZE;
  return_value = device_read(g_rxbuf, g_nrxcount, true);
  if (return_value == 0)
  {
    CFrameInfo fi(g_rxbuf, g_nrxcount);

    uint32_t bytesextracted;
    bytesextracted = pno.Extractseupno(fi.PPnoBody());

    if (!bytesextracted)
    {
      ROS_ERROR("[POLHEMUS] No bytes were extracted by pno.Extractseupno");
      return_value = RETURN_ERROR;
      return return_value;
    }
    // bitmap of active sensors
    sensor_map = pno.SensorMap();

    // update this as a sensor may have been dropped
    station_count = pno.SensorCount();
    return station_count;
  }
  return RETURN_ERROR;
}

int Viper::fill_pno_data(geometry_msgs::TransformStamped *transform, int &index)
{
  // Set translation (in metres)
  int return_value = 0;

  transform->child_frame_id = "polhemus_station_" +
    std::to_string(pno.SensFrame(index)->SFinfo.bfSnum);
  transform->transform.translation.x = pno.SensFrame(index)->pno.pos[0];
  transform->transform.translation.y = pno.SensFrame(index)->pno.pos[1];
  transform->transform.translation.z = pno.SensFrame(index)->pno.pos[2];
  // Set rotation
  transform->transform.rotation.w = pno.SensFrame(index)->pno.ori[0];
  transform->transform.rotation.x = pno.SensFrame(index)->pno.ori[1];
  transform->transform.rotation.y = pno.SensFrame(index)->pno.ori[2];
  transform->transform.rotation.z = pno.SensFrame(index)->pno.ori[3];

  index = pno.SensFrame(index)->SFinfo.bfSnum;

  return return_value;
}

int Viper::define_data_type(data_type_e data_type)
{
  int return_value = RETURN_ERROR;

  viper_cmds_e cmd_type = CMD_UNITS;
  viper_cmd_actions_e action = CMD_ACTION_SET;

  viper_units_config_t cfg;

  if (data_type == DATA_TYPE_QUAT)
  {
    cfg.ori_units = ORI_QUATERNION;
  }
  else if (data_type == DATA_TYPE_EULER)
  {
    cfg.ori_units = ORI_EULER_DEGREE;
  }
  else
  {
    return return_value;
  }

  cfg.pos_units = POS_METER;
  CVPcmd viper_command;
  viper_command.Fill(cmd_type, action, -1, 0, &cfg, sizeof(cfg));
  viper_command.Prepare(g_txbuf, g_ntxcount);
  int nBytes = g_ntxcount;
  uint8_t *pbuf = g_txbuf;
  return_value = device_send(pbuf, nBytes);
  if (return_value == 0)
  {
    return_value = receive_data_frame(cmd_type);
  }

  return return_value;
}

int Viper::request_num_of_stations(void)
{
  int return_value = RETURN_ERROR;
  viper_cmds_e cmd_type = CMD_STATION_MAP;
  viper_cmd_actions_e action = CMD_ACTION_GET;
  CVPcmd viper_command;

  CStationMap cstamap;
  viper_command.Fill(cmd_type, action);

  viper_command.Prepare(g_txbuf, g_ntxcount);
  int nBytes = g_ntxcount;
  uint8_t *pbuf = g_txbuf;

  return_value = device_send(pbuf, nBytes);

  if (return_value == 0)
  {
    g_nrxcount = VIPER_RX_BUF_SIZE;
    return_value = device_read(g_rxbuf, g_nrxcount, true);

    if (return_value == 0)
    {
      CFrameInfo fi(g_rxbuf, g_nrxcount);

      if (fi.cmd() == CMD_STATION_MAP)
      {
        CStationMap cmap(reinterpret_cast<viper_station_map_t*>
          (fi.PCmdPayload()));
        cstamap = cmap;
      }
    }
  }

  station_count = cstamap.SnsDetectedCount();
  return return_value;
}

/* sets the zenith of the hemisphere in direction of vector (x, y, z) */
int Viper::set_hemisphere(int x, int y, int z)
{
  int return_value = RETURN_ERROR;
  viper_cmds_e cmd_type = CMD_HEMISPHERE;
  viper_cmd_actions_e action = CMD_ACTION_SET;
  viper_hemisphere_config_t cfg;
  cfg.track_enabled = 0;
  cfg.params[0] = x;
  cfg.params[1] = y;
  cfg.params[2] = z;
  CVPcmd viper_command;
  viper_command.Fill(cmd_type, action, -1, 0, &cfg, sizeof(cfg));
  viper_command.Prepare(g_txbuf, g_ntxcount);
  int nBytes = g_ntxcount;
  uint8_t *pbuf = g_txbuf;
  return_value = device_send(pbuf, nBytes);
  if (return_value == 0)
  {
    return_value = receive_data_frame(cmd_type);
  }
  return return_value;
}

int Viper::set_boresight(bool reset_origin, int station,
    float arg_1, float arg_2, float arg_3, float arg_4)
{
  int return_value = RETURN_ERROR;
  viper_cmds_e cmd_type = CMD_BORESIGHT;
  viper_cmd_actions_e action = CMD_ACTION_SET;
  viper_boresight_config_t cfg;
  cfg.params[0] = arg_1;
  cfg.params[1] = arg_2;
  cfg.params[2] = arg_3;
  cfg.params[3] = arg_4;
  CVPcmd viper_command;

  viper_command.Fill(cmd_type, action, station,
    reset_origin, &cfg, sizeof(cfg));
  viper_command.Prepare(g_txbuf, g_ntxcount);

  int nBytes = g_ntxcount;
  uint8_t *pbuf = g_txbuf;
  return_value = device_send(pbuf, nBytes);
  if (return_value == 0)
  {
    return_value = receive_data_frame(cmd_type);
  }
  return return_value;
}

int Viper::reset_boresight(void)
{
  int return_value = RETURN_ERROR;
  viper_cmds_e cmd_type = CMD_BORESIGHT;
  viper_cmd_actions_e action = CMD_ACTION_RESET;

  CVPcmd viper_command;

  viper_command.Fill(cmd_type, action, -1);
  viper_command.Prepare(g_txbuf, g_ntxcount);

  int nBytes = g_ntxcount;
  uint8_t *pbuf = g_txbuf;
  return_value = device_send(pbuf, nBytes);

  if (return_value == 0)
  {
    return_value = receive_data_frame(cmd_type);
  }

  return return_value;
}

tf2::Quaternion Viper::get_station_quaternion(int station_id)
{
  if (pno.SensFrame(station_id) != NULL)
  {
    tf2::Quaternion q(
        pno.SensFrame(station_id)->pno.ori[1],
        pno.SensFrame(station_id)->pno.ori[2],
        pno.SensFrame(station_id)->pno.ori[3],
        pno.SensFrame(station_id)->pno.ori[0]);
    return q;
  }
  else
  {
    throw std::runtime_error(
      "Could not retrieve current sensor orientation, empty sensor frame!");
  }
}

int Viper::set_source(int source, int station_id)
{
  int return_value = RETURN_ERROR;
  viper_cmds_e cmd_type = CMD_SRC_SELECT;
  viper_cmd_actions_e action = CMD_ACTION_SET;
  viper_src_select_cfg_t cfg;
  if (source < 1)
  {
    ROS_WARN("[POLHEMUS] Failed to select source, source value must > 0");
    return return_value;
  }
  source = 1 << source - 1;
  cfg.src_select_map = source;
  CVPcmd viper_command;

  viper_command.Fill(cmd_type, action, station_id, 0, &cfg, sizeof(cfg));
  viper_command.Prepare(g_txbuf, g_ntxcount);

  int nBytes = g_ntxcount;
  uint8_t *pbuf = g_txbuf;
  return_value = device_send(pbuf, nBytes);
  if (return_value == 0)
  {
    return_value = receive_data_frame(cmd_type);
  }
  return return_value;
}

bool Viper::persist_commands(void)
{
  int return_value = RETURN_ERROR;
  viper_cmds_e cmd_type = CMD_PERSIST;
  viper_cmd_actions_e action = CMD_ACTION_SET;

  CVPcmd viper_command;

  viper_command.Fill(cmd_type, action);
  viper_command.Prepare(g_txbuf, g_ntxcount);

  int nBytes = g_ntxcount;
  uint8_t *pbuf = g_txbuf;
  return_value = device_send(pbuf, nBytes);
  if (return_value == 0)
  {
    return_value = receive_data_frame(cmd_type);
  }

  if (return_value == 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

int Viper::send_saved_calibration()
{
  int return_value = RETURN_ERROR;
  return_value = set_device_to_receive_saved_calibration();
  if (RETURN_ERROR == return_value)
    return -1;

  // send the calibration saved in calibration.yaml
  // read from param server the x, y and z for all stations, so we need to loop stations and send boresight command
  for (int station_number = 0; station_number < station_count; ++station_number)
  {
    int station_id = station_number;
    if (pno.SensFrame(station_number) != NULL)
    {
      station_id = pno.SensFrame(station_number)->SFinfo.bfSnum;
    }
    else
    {
      ROS_WARN("No pno frame for station, this is normal if there is no previous calibration "
        "(boresight) data avaliable.");
    }

    if (!nh->hasParam(name + "_calibration/rotations/station_" + std::to_string(station_id)))
    {
      ROS_WARN("[POLHEMUS] No previous calibration (boresight) data available, please calibrate before proceeding!!!");
      break;
    }

    ROS_INFO("[POLHEMUS] Reading saved calibration (boresight) data and calibrating station %d...", station_id);

    // retrieve calibration angles
    float calibrated_roll;
    std::string calibrated_roll_param_name = name +
      "_calibration/rotations/station_" + std::to_string(station_id) +
      "/calibrated_roll";
    nh->getParam(calibrated_roll_param_name, calibrated_roll);

    float calibrated_pitch;
    std::string calibrated_pitch_param_name = name +
      "_calibration/rotations/station_" + std::to_string(station_id) +
      "/calibrated_pitch";
    nh->getParam(calibrated_pitch_param_name, calibrated_pitch);

    float calibrated_yaw;
    std::string calibrated_yaw_param_name = name +
      "_calibration/rotations/station_" + std::to_string(station_id) +
      "/calibrated_yaw";
    nh->getParam(calibrated_yaw_param_name, calibrated_yaw);

    // retrieve current sensor orientation
    tf2::Quaternion station_quaternion;
    try
    {
      station_quaternion = get_station_quaternion(station_number);
    }
    catch(std::runtime_error &error)
    {
      ROS_ERROR_STREAM("Caught runtime error for station " << station_id << ": " << error.what());
      throw;
    }

    double station_roll, station_pitch, station_yaw;
    tf2::Matrix3x3(station_quaternion).getRPY(station_roll, station_pitch, station_yaw);

    station_roll = (station_roll * 180) / PI;
    station_pitch = (station_pitch * 180) / PI;
    station_yaw = (station_yaw * 180) / PI;

    // compute correction needed to calibrate sensor to 0
    float correction_roll = station_roll - calibrated_roll;
    float correction_pitch = station_pitch - calibrated_pitch;
    float correction_yaw = station_yaw - calibrated_yaw;

    define_data_type(DATA_TYPE_EULER);
    return_value = set_boresight(false, station_id, correction_yaw, correction_pitch, correction_roll);
    define_data_type(DATA_TYPE_QUAT);

    if (RETURN_ERROR == return_value)
    {
      ROS_ERROR("[POLHEMUS] Error sending calibration (boresight) from file.");
      return -1;
    }
  }

  device_data_mode(DATA_CONTINUOUS);
  return 0;
}

bool Viper::calibrate(std::string boresight_calibration_file)
{
  int return_value = RETURN_ERROR;

  // set data mode to single to allow correct boresight reset
  device_data_mode(DATA_SINGLE);

  return_value = set_device_for_calibration();
  if (RETURN_ERROR == return_value)
    return -1;

  for (int station_number = 0; station_number < station_count; ++station_number)
  {
    int station_id = pno.SensFrame(station_number)->SFinfo.bfSnum;
    save_current_calibration_to_file(station_id, station_number);
  }

  std::string cmd("rosparam dump ");
  cmd += boresight_calibration_file + " /calibration";

  int dump_calibration_param_status = system(cmd.c_str());
  if (dump_calibration_param_status < 0)
  {
    ROS_ERROR("[POLHEMUS] Error saving calibration.");
    return -1;
  }
  ROS_INFO("[POLHEMUS] Calibration file saved at: %s\n", boresight_calibration_file.c_str());

  define_data_type(DATA_TYPE_EULER);
  return_value = set_boresight(false, -1, 0, 0, 0);
  define_data_type(DATA_TYPE_QUAT);

  if (RETURN_ERROR == return_value)
  {
    ROS_ERROR("[POLHEMUS] Calibration failed.");
  }

  // set data mode back to continuous
  return_value = device_data_mode(DATA_CONTINUOUS);
  if (RETURN_ERROR == return_value)
  {
    ROS_ERROR("[POLHEMUS] Setting data mode to continuous, failed.\n");
    return return_value;
  }

  return true;
}

bool Viper::src_select_srv(polhemus_ros_driver::set_source::Request &req,
    polhemus_ros_driver::set_source::Response &res)
{
  ROS_INFO("[POLHEMUS] Set source request...");
  if (set_source(req.source, req.sensor))
  {
    res.success = false;
  }
  else
  {
    res.success = true;
  }
  return true;
}

bool Viper::persist_srv(polhemus_ros_driver::persist::Request &req, polhemus_ros_driver::persist::Response &res)
{
  ROS_INFO("[POLHEMUS] Making config persistent");
  res.success = persist_commands();
  return true;
}
