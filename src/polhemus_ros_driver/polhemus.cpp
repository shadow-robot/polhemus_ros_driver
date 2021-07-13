/*
* Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
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


Polhemus::Polhemus(std::string name, uint16_t rx_buffer_size, uint16_t tx_buffer_size):
  name(name), rx_buffer_size(rx_buffer_size), tx_buffer_size(tx_buffer_size), g_rxbuf(new uint8_t[rx_buffer_size]),
  g_txbuf(new uint8_t[tx_buffer_size])
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

int Polhemus::device_send(uint8_t *cmd, int &count)
{
  int retval = device_write(cmd, count, TIMEOUT);
  if (RETURN_ERROR == retval)
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
  while (g_nrxcount > 0)
  {
    device_read(g_rxbuf, g_nrxcount, true);
  }
}

int Polhemus::set_device_to_receive_saved_calibration(int number_of_hands)
{
  int retval = RETURN_ERROR;
  int required_number_of_sensors = number_of_hands*SENSORS_PER_GLOVE;

  if (nh->hasParam(name + "_calibration/rotations"))
  {
    retval = receive_pno_data_frame();
    ros::Time start_time = ros::Time::now();

    while (retval < required_number_of_sensors)
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
  return 1;
}

int Polhemus::set_device_for_calibration(void)
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
      ROS_ERROR("[POLHEMUS] Calibration - error getting complete frame in required time.");
      return -1;
    }
  }

  device_reset();
  return retval;
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
