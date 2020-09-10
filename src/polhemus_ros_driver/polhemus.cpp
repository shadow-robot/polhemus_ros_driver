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
    reset_boresight();
    retval = receive_pno_data_frame();

    device_reset();

  }
  else
  {
    ROS_WARN("[POLHEMUS] No previous calibration data available, please calibrate before proceeding!!!");
    return 0;
  }

  // send the calibration saved in calibration.yaml
  // read from param server the x, y and z for all stations, so we need to loop stations and send boresight command
  for (int i = 0; i < station_count; ++i)
  {
    if (nh->hasParam("/calibration/" + name + "_calibration/rotations/station_" + std::to_string(i)))
    {
      std::string x_name = "/calibration/" + name + "_calibration/rotations/station_" + std::to_string(i) + "/x";
      std::string y_name = "/calibration/" + name + "_calibration/rotations/station_" + std::to_string(i) + "/y";
      std::string z_name = "/calibration/" + name + "_calibration/rotations/station_" + std::to_string(i) + "/z";
      float x;
      float y;
      float z;

      // read x y and z rotations from param server
      nh->getParam(x_name, x);
      nh->getParam(y_name, y);
      nh->getParam(z_name, z);

      if (x == 0 & y == 0 & z == 0)
      {
        // no previous calibration exists
        ROS_WARN("[POLHEMUS] No previous calibration data available, please calibrate before proceeding!!!");
        break;
      }
      else
      {
        ROS_INFO("[POLHEMUS] Calibrating station %d.", i);

        if (name == "viper")
        {
          retval = define_data_type(DATA_TYPE_EULER);
        }
        else
        {
          retval = 0;
        }
        if (!retval)
        {
          tf2::Quaternion q = get_quaternion(i);
          double roll, pitch, yaw;
          tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

          roll = (roll * 180) / PI;
          pitch = (pitch * 180) / PI;
          yaw = (yaw * 180) / PI;

          x = roll - x;
          y = pitch - y;
          z = yaw - z;

          if (name == "viper")
          {
            retval = set_boresight(false, i, z, y, x);
          }
          else
          {
            retval = set_boresight(false, i + 1, z, y, x);
          }

          if (retval == RETURN_ERROR)
          {
            ROS_ERROR("[POLHEMUS] Error sending calibration from file.");
            break;
          }
        }
      }
    }
    else
    {
      // no previous calibration exists
      ROS_WARN("[POLHEMUS] Station could not be found in calibration, please calibrate before proceeding!!!");
      break;
    }
  }
  if (name == "viper")
  {
    define_data_type(DATA_TYPE_QUAT);
  }

  device_data_mode(DATA_CONTINUOUS);
  return 0;
}

bool Polhemus::calibrate(std::string boresight_calibration_file)
{
  bool retval = false;

  reset_boresight();

  retval = receive_pno_data_frame();
  retval = receive_pno_data_frame();

  device_reset();

  if (name == "viper")
  {
    define_data_type(DATA_TYPE_EULER);
  }

  for (int i = 0; i < station_count; ++i)
  {
    tf2::Quaternion q = get_quaternion(i);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // convert to degrees
    roll = (roll * 180) / 3.14;
    pitch = (pitch * 180) / 3.14;
    yaw = (yaw * 180) / 3.14;

    ROS_INFO("[POLHEMUS] Calibrating station %d.", i);

    // save values to config file
    std::string x_name = "/calibration/" + name + "_calibration/rotations/station_" + std::to_string(i) + "/x";
    std::string y_name = "/calibration/" + name + "_calibration/rotations/station_" + std::to_string(i) + "/y";
    std::string z_name = "/calibration/" + name + "_calibration/rotations/station_" + std::to_string(i) + "/z";

    nh->setParam(x_name, roll);
    nh->setParam(y_name, pitch);
    nh->setParam(z_name, yaw);
  }


  std::string cmd("rosparam dump ");
  cmd += boresight_calibration_file + " /calibration";
  
  if (system(cmd.c_str()))
  {
    ROS_ERROR("[POLHEMUS] Error saving calibration.");
  }
  else
  {
    ROS_INFO("[POLHEMUS] Calibration file saved at: %s\n", boresight_calibration_file.c_str());
  }

  int ret = set_boresight(false, -1, 0, 0, 0);

  if (ret == RETURN_ERROR)
  {
    ROS_ERROR("[POLHEMUS] Calibration failed.");
  }

  if (name == "viper")
  {
    ret = define_data_type(DATA_TYPE_QUAT);
    if (ret == RETURN_ERROR)
    {
      ROS_ERROR("[POLHEMUS] Setting data type to quaternion, failed.\n");
      return retval;
    }
  }

  // set data mode back to continuous
  ret = device_data_mode(DATA_CONTINUOUS);
  if (ret == RETURN_ERROR)
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

int Polhemus::fill_pno_data(geometry_msgs::TransformStamped *transform, int station_id)
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

int Polhemus::set_source(int source)
{
}

tf2::Quaternion Polhemus::get_quaternion(int index)
{
}
