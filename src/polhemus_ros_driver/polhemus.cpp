/*
* Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/


#include <string.h>
#include <libusb-1.0/libusb.h>
#include "polhemus_ros_driver/polhemus.hpp"
#include <ros/console.h>


#ifdef DEBUG
#include <stdio.h>
#define warn(as...) { fprintf(stderr, "%s:%d: ", __FILE__, __LINE__); \
    fprintf(stderr, as); }
#else
#define warn(as...)
#endif


Polhemus::Polhemus(std::string name, uint16_t rx_buffer_size, uint16_t tx_buffer_size):
  name(name), rx_buffer_size(rx_buffer_size), tx_buffer_size(tx_buffer_size), g_rxbuf(new uint8_t[rx_buffer_size]),
  g_txbuf(new uint8_t[tx_buffer_size])
{
}

Polhemus::~Polhemus(void)
{
}

int Polhemus::count_bits(uint16_t v) {
  int c;
  for (c = 0; v; c++)
    {
      v &= v - 1; // clear the least significant bit set
    }
  return c;
}

int Polhemus::device_write(uint8_t *buf, int size, int timeout)
{
  int nActual = 0;
  int retval = RETURN_ERROR;

  retval = libusb_bulk_transfer(device_handle, endpoint_out, buf, size, &nActual, timeout);

  if (retval != 0)
  {
    ROS_ERROR("[POLHEMUS] USB write failed with code %d.", retval);
    retval = RETURN_ERROR;
  }
  else if (nActual != size)
  {
    size = nActual;
    retval = RETURN_ERROR;
    ROS_ERROR("[POLHEMUS] write count wrong.\n\n");
  }
  else if ((nActual % endpoint_out_max_packet_size) == 0)
  {
    retval = RETURN_ERROR;
    ROS_ERROR("[POLHEMUS] Device write, size larger than max packet size.");
    libusb_bulk_transfer(device_handle, endpoint_out, nullptr, 0, &nActual, timeout);
  }

  return retval;
}

int Polhemus::device_init(void)
{
  if (libusb_set_configuration(device_handle, CONFIGURATION) != 0)
    ROS_ERROR("[POLHEMUS] Could not set usb configuration to %d", CONFIGURATION);

  if (libusb_claim_interface(device_handle, INTERFACE) != 0)
  {
    ROS_ERROR("[POLHEMUS] Could not claim usb interface %d", INTERFACE);
    return RETURN_ERROR;
  }

  device_reset();
  return 0;
}

int Polhemus::device_send(uint8_t *cmd, int &count)
{
  if (device_write(cmd, count, TIMEOUT))
  {
    ROS_WARN("[POLHEMUS] Sending cmd `%d' to device failed", *cmd);
    return RETURN_ERROR;
  }
  return 0;
}

int Polhemus::device_read(void *pbuf, int &size, bool bTOisErr)
{
  uint32_t timeout = VPUSB_READ_TIMEOUT_MS;
  int retval = RETURN_ERROR;
  int nActual = 0;
  unsigned char *pbuf_c = (unsigned char*) pbuf;

  retval = libusb_bulk_transfer(device_handle, endpoint_in, pbuf_c, size, &nActual, timeout);

  if ((retval == LIBUSB_ERROR_TIMEOUT) && bTOisErr)
  {
    retval = RETURN_ERROR;
    size = 0;
  }
  else
  {
    size = nActual;
    retval = 0;
  }

  return retval;
}

void Polhemus::device_clear_input(void)
{
  g_nrxcount = rx_buffer_size;
  while(g_nrxcount > 0)
  {
    device_read(g_rxbuf, g_nrxcount, true);
  }
}

int Polhemus::send_saved_calibration(void)
{
  int retval = RETURN_ERROR;
  if (nh->hasParam("/calibration/" + name + "_calibration/rotations"))
  {
    retval = receive_pno_data_frame();
    ros::Time start_time = ros::Time::now();

    while (retval < SENSORS_PER_GLOVE)
    {
      retval = receive_pno_data_frame();
      if (ros::Time::now().toSec() - start_time.toSec() >= CALIBRATE_TIMEOUT_IN_SECS)
      {
        ROS_ERROR("[POLHEMUS] Calibration - error getting complete frame in required time.");
        return -1;
      }
    }

    device_reset();
  }
  else
  {
    ROS_WARN("[POLHEMUS] No previous calibration data available, please calibrate before proceeding!!!");
    return 0;
  }

  // send the calibration saved in calibration.yaml
  // read from param server the x, y and z for all stations, so we need to loop stations and send boresight command
  for (int station_id = 0; station_id < station_count; ++station_id)
  {
    if (!nh->hasParam("/calibration/" + name + "_calibration/rotations/station_" + std::to_string(station_id)))
    {
      ROS_WARN("[POLHEMUS] No previous calibration data available, please calibrate before proceeding!!!");
      break;
    }
    
    ROS_INFO("[POLHEMUS] Reading saved calibration data and calibrating station %d...", station_id);

    // retrieve calibration angles
    float calibrated_roll;
    std::string calibrated_roll_param_name = "/calibration/" + name + "_calibration/rotations/station_" + std::to_string(station_id) + "/calibrated_roll";
    nh->getParam(calibrated_roll_param_name, calibrated_roll);

    float calibrated_pitch;
    std::string calibrated_pitch_param_name = "/calibration/" + name + "_calibration/rotations/station_" + std::to_string(station_id) + "/calibrated_pitch";
    nh->getParam(calibrated_pitch_param_name, calibrated_pitch);

    float calibrated_yaw;
    std::string calibrated_yaw_param_name = "/calibration/" + name + "_calibration/rotations/station_" + std::to_string(station_id) + "/calibrated_yaw";
    nh->getParam(calibrated_yaw_param_name, calibrated_yaw);

    // retrieve current sensor orientation
    tf2::Quaternion station_quaternion;
    try
    {
      station_quaternion = get_station_quaternion(station_id);
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

    if (name == "viper")
    {
      define_data_type(DATA_TYPE_EULER);
      retval = set_boresight(false, station_id, correction_yaw, correction_pitch, correction_roll);
      define_data_type(DATA_TYPE_QUAT);
    }
    else
    {
      retval = set_boresight(false, station_id + 1, correction_yaw, correction_pitch, correction_roll);
    }

    if (RETURN_ERROR == retval)
    {
      ROS_ERROR("[POLHEMUS] Error sending calibration from file.");
      return -1;
    }
  }

  device_data_mode(DATA_CONTINUOUS);
  return 0;
}

bool Polhemus::calibrate(std::string boresight_calibration_file)
{
  int retval = RETURN_ERROR;

  reset_boresight();

  retval = receive_pno_data_frame();
  ros::Time start_time = ros::Time::now();

  while (retval < SENSORS_PER_GLOVE)
  {
    retval = receive_pno_data_frame();
    if (ros::Time::now().toSec() - start_time.toSec() >= CALIBRATE_TIMEOUT_IN_SECS)
    {
      ROS_ERROR("[POLHEMUS] Caliration - error getting complete frame in required time.");
      return -1;
    }
  }

  device_reset();

  for (int station_id = 0; station_id < station_count; ++station_id)
  {
    ROS_INFO("[POLHEMUS] Calibrating station %d.", station_id);

    tf2::Quaternion station_quaternion = get_station_quaternion(station_id);

    double station_roll, station_pitch, station_yaw;
    tf2::Matrix3x3(station_quaternion).getRPY(station_roll, station_pitch, station_yaw);

    // convert to degrees
    station_roll = (station_roll * 180) / PI;
    station_pitch = (station_pitch * 180) / PI;
    station_yaw = (station_yaw * 180) / PI;

    // save values to config file
    std::string calibrated_roll_param_name = "/calibration/" + name + "_calibration/rotations/station_" + std::to_string(station_id) + "/calibrated_roll";
    nh->setParam(calibrated_roll_param_name, station_roll);

    std::string calibrated_pitch_param_name = "/calibration/" + name + "_calibration/rotations/station_" + std::to_string(station_id) + "/calibrated_pitch";
    nh->setParam(calibrated_pitch_param_name, station_pitch);
  
    std::string calibrated_yaw_param_name = "/calibration/" + name + "_calibration/rotations/station_" + std::to_string(station_id) + "/calibrated_yaw";
    nh->setParam(calibrated_yaw_param_name, station_yaw);
  }

  std::string cmd("rosparam dump ");
  cmd += boresight_calibration_file + " /calibration";
  
  int dump_calibration_param_status = system(cmd.c_str());
  if (dump_calibration_param_status < 0)
  { 
    ROS_ERROR("[POLHEMUS] Error saving calibration.");
    return -1;
  }
  else
  {
    ROS_INFO("[POLHEMUS] Calibration file saved at: %s\n", boresight_calibration_file.c_str());

  }

  define_data_type(DATA_TYPE_EULER);
  retval = set_boresight(false, -1, 0, 0, 0);
  define_data_type(DATA_TYPE_QUAT);

  if (RETURN_ERROR == retval)
  {
    ROS_ERROR("[POLHEMUS] Calibration failed.");
  }

  // set data mode back to continuous
  retval = device_data_mode(DATA_CONTINUOUS);
  if (RETURN_ERROR == retval)
  {
    ROS_ERROR("[POLHEMUS] Setting data mode to continuous, failed.\n");
    return retval;
  }

  return true;
}

bool Polhemus::calibrate_srv(polhemus_ros_driver::calibrate::Request &req, polhemus_ros_driver::calibrate::Response &res, std::string boresight_calibration_file)
{
  ROS_INFO("[POLHEMUS] Calibration request...");
  res.success = calibrate(boresight_calibration_file);
  return true;
}

bool Polhemus::src_select_srv(polhemus_ros_driver::set_source::Request &req, polhemus_ros_driver::set_source::Response &res)
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

bool Polhemus::persist_srv(polhemus_ros_driver::persist::Request &req, polhemus_ros_driver::persist::Response &res)
{
  ROS_INFO("[POLHEMUS] Making config persistent");
  res.success = persist_commands();
  return true;
}

int Polhemus::device_reset(void)
{
}

int Polhemus::request_num_of_stations(void)
{
}

int Polhemus::set_hemisphere(int x, int y, int z)
{
}

int Polhemus::define_data_type(data_type_e data_type)
{
}

int Polhemus::device_binary_mode(void)
{
}

void Polhemus::generate_data_structure(void)
{
}

int Polhemus::receive_pno_data_frame(void)
{
}

int Polhemus::fill_pno_data(geometry_msgs::TransformStamped *transform, int &station_id)
{
}

int Polhemus::device_data_mode(data_mode_e mode)
{
}

int Polhemus::set_boresight(bool reset_origin, int station, float arg_1, float arg_2, float arg_3, float arg_4)
{
}

int Polhemus::reset_boresight(void)
{
}

bool Polhemus::persist_commands(void)
{
}

int Polhemus::set_source(int source, int station_id)
{
}

tf2::Quaternion Polhemus::get_station_quaternion(int station_id)
{
}
