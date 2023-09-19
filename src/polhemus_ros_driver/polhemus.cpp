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


#include <string>
#include <libusb-1.0/libusb.h>
#include "polhemus_ros_driver/polhemus.hpp"
#include <ros/console.h>


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


Polhemus::Polhemus(std::string name, uint16_t rx_buffer_size, uint16_t tx_buffer_size,
                   uint8_t sensors_right_glove, uint8_t sensors_left_glove):
  name(name), rx_buffer_size(rx_buffer_size), tx_buffer_size(tx_buffer_size), g_rxbuf(new uint8_t[rx_buffer_size]),
  g_txbuf(new uint8_t[tx_buffer_size]), sensors_right_glove(sensors_right_glove), sensors_left_glove(sensors_left_glove)
{
}

Polhemus::~Polhemus(void)
{
}

int Polhemus::count_bits(uint16_t v)
{
  int c;
  for (c = 0; v; c++)
    {
      v &= v - 1;  // clear the least significant bit set
    }
  return c;
}

int Polhemus::device_write(uint8_t *buf, int size, int timeout)
{
  int nActual = 0;
  int return_value = RETURN_ERROR;

  return_value = libusb_bulk_transfer(device_handle, endpoint_out, buf, size, &nActual, timeout);

  if (return_value != 0)
  {
    ROS_ERROR("[POLHEMUS] USB write failed with code %d.", return_value);
    return_value = RETURN_ERROR;
  }
  else if (nActual != size)
  {
    size = nActual;
    return_value = RETURN_ERROR;
    ROS_ERROR("[POLHEMUS] write count wrong.\n\n");
  }
  else if ((nActual % endpoint_out_max_packet_size) == 0)
  {
    return_value = RETURN_ERROR;
    ROS_ERROR("[POLHEMUS] Device write, size larger than max packet size.");
    libusb_bulk_transfer(device_handle, endpoint_out, nullptr, 0, &nActual, timeout);
  }

  return return_value;
}

int Polhemus::device_send(uint8_t *cmd, int &count)
{
  int return_value = device_write(cmd, count, TIMEOUT);
  if (RETURN_ERROR == return_value)
  {
    ROS_WARN("[POLHEMUS] Sending cmd `%d' to device failed", *cmd);
    return RETURN_ERROR;
  }
  return 0;
}

int Polhemus::device_read(void *pbuf, int &size, bool bTOisErr)
{
  uint32_t timeout = VPUSB_READ_TIMEOUT_MS;
  int return_value = RETURN_ERROR;
  int nActual = 0;
  unsigned char *pbuf_c = (unsigned char*) pbuf;

  return_value = libusb_bulk_transfer(device_handle, endpoint_in, pbuf_c, size, &nActual, timeout);



  if (return_value != LIBUSB_SUCCESS)
  {
    if (latest_usb_read_result_ != return_value)
    {
      ROS_WARN("[POLHEMUS] USB read failed with code %d. Error: %s", return_value,
        libusb_strerror(static_cast<libusb_error>(return_value)));
    }
    else
    {
      // Print the warning at most every 5 seconds to avoid flooding the console with the same message
      ROS_WARN_THROTTLE(5, "[POLHEMUS] USB read failed with code %d. Error: %s", return_value,
        libusb_strerror(static_cast<libusb_error>(return_value)));
    }
  }

  latest_usb_read_result_ = return_value;

  if ((return_value == LIBUSB_ERROR_TIMEOUT) && bTOisErr)
  {
    return_value = RETURN_ERROR;
    size = 0;
  }
  else
  {
    size = nActual;
    return_value = 0;
  }

  return return_value;
}

void Polhemus::device_clear_input(void)
{
  g_nrxcount = rx_buffer_size;
  while (g_nrxcount > 0)
  {
    device_read(g_rxbuf, g_nrxcount, true);
  }
}

int Polhemus::set_device_to_receive_saved_calibration()
{
  int return_value = RETURN_ERROR;
  uint16_t required_number_of_sensors = sensors_right_glove + sensors_left_glove;

  if (nh->hasParam(name + "_calibration/rotations"))
  {
    return_value = receive_pno_data_frame();
    ros::Time start_time = ros::Time::now();

    while (return_value < required_number_of_sensors)
    {
      return_value = receive_pno_data_frame();
      if (ros::Time::now().toSec() - start_time.toSec() >= CALIBRATE_TIMEOUT_IN_SECS)
      {
        ROS_ERROR("[POLHEMUS] Calibration (boresight) - error getting complete frame in required time.");
        return -1;
      }
    }
    device_reset();
  }
  else
  {
    ROS_WARN("[POLHEMUS] No previous calibration (boresight) data available, please boresight before proceeding!!!");
    return 0;
  }
  return 1;
}

int Polhemus::set_device_for_calibration(void)
{
  int return_value = RETURN_ERROR;
  uint16_t required_number_of_sensors = sensors_right_glove + sensors_left_glove;

  reset_boresight();

  return_value = receive_pno_data_frame();
  ros::Time start_time = ros::Time::now();

  while (return_value < required_number_of_sensors)
  {
    return_value = receive_pno_data_frame();
    if (ros::Time::now().toSec() - start_time.toSec() >= CALIBRATE_TIMEOUT_IN_SECS)
    {
      ROS_ERROR("[POLHEMUS] Calibration (boresight) - error getting complete frame in required time.");
      return -1;
    }
  }

  device_reset();
  return return_value;
}

void Polhemus::save_current_calibration_to_file(int station_id, int station_number)
{
  ROS_INFO("[POLHEMUS] Calibrating station %d.", station_id);

  tf2::Quaternion station_quaternion = get_station_quaternion(station_number);

  double station_roll, station_pitch, station_yaw;
  tf2::Matrix3x3(station_quaternion).getRPY(station_roll, station_pitch, station_yaw);

  // convert to degrees
  station_roll = (station_roll * 180) / PI;
  station_pitch = (station_pitch * 180) / PI;
  station_yaw = (station_yaw * 180) / PI;

  // save values to config file
  std::string calibrated_roll_param_name = "/calibration/" + name +
    "_calibration/rotations/station_" + std::to_string(station_id) + "/calibrated_roll";
  nh->setParam(calibrated_roll_param_name, station_roll);

  std::string calibrated_pitch_param_name = "/calibration/" + name +
    "_calibration/rotations/station_" + std::to_string(station_id) + "/calibrated_pitch";
  nh->setParam(calibrated_pitch_param_name, station_pitch);

  std::string calibrated_yaw_param_name = "/calibration/" + name +
    "_calibration/rotations/station_" + std::to_string(station_id) + "/calibrated_yaw";
  nh->setParam(calibrated_yaw_param_name, station_yaw);
}

bool Polhemus::calibrate_srv(polhemus_ros_driver::calibrate::Request &req,
    polhemus_ros_driver::calibrate::Response &res, std::string boresight_calibration_file)
{
  ROS_INFO("[POLHEMUS] Calibration request...");
  res.success = calibrate(boresight_calibration_file);
  return true;
}
