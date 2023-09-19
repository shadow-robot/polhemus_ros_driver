/*

 Copyright (C) 2022 Shadow Robot Company Ltd <software@shadowrobot.com>

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


#ifndef POLHEMUS_H
#define POLHEMUS_H

#include <ros/ros.h>
#include <ros/console.h>
#include <libusb-1.0/libusb.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <geometry_msgs/TransformStamped.h>
#include "polhemus_ros_driver/calibrate.h"
#include "polhemus_ros_driver/set_source.h"
#include "polhemus_ros_driver/persist.h"
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <yaml.h>


#define INTERFACE 0
#define CONFIGURATION 1
#define TIMEOUT 1000
#define CRC_BYTES 4
#define SEUID 0
#define VPUSB_WRITE_TIMEOUT_MS 100
#define VPUSB_READ_TIMEOUT_MS 100
#define PI 3.14159265359
#define CALIBRATE_TIMEOUT_IN_SECS 3
#define RETURN_ERROR -1

/* make control character out of ordinary character */
#define control(c) ((c) & 0x1f)

typedef enum data_mode_e
{
    DATA_CONTINUOUS,
    DATA_SINGLE,
    DATA_RESET
}
data_mode_e;

typedef enum data_type_e
{
    DATA_TYPE_QUAT,
    DATA_TYPE_EULER
}
data_type_e;

class Polhemus
{
public:
  Polhemus(std::string name, uint16_t rx_buffer_size, uint16_t tx_buffer_size,
           uint8_t sensors_right_glove, uint8_t sensors_left_glove);
  virtual ~Polhemus(void);
  int count_bits(uint16_t v);
  ros::NodeHandle *nh;

  /* set up usb interface and configuration, send initial magic and reset */
  virtual void device_init(void) = 0;
  virtual int device_data_mode(data_mode_e mode) = 0;
  /* send a command */
  int device_send(uint8_t *cmd, int &count);
  /* read until the device doesn't send anything else */
  void device_clear_input(void);
  virtual int receive_pno_data_frame(void) = 0;
  virtual int fill_pno_data(geometry_msgs::TransformStamped *transform, int &index) = 0;

  int device_read(void *pbuf, int &count, bool bTOisErr/*=false*/);
  int device_write(uint8_t *buf, int size, int timeout);
  virtual int device_reset(void) = 0;
  virtual int define_data_type(data_type_e data_type) = 0;
  virtual int set_hemisphere(int x, int y, int z) = 0;
  virtual int request_num_of_stations(void) = 0;
  virtual int set_boresight(bool reset_origin, int station, float arg_1, float arg_2, float arg_3, float arg_4 = 0) = 0;
  virtual int reset_boresight(void) = 0;
  virtual tf2::Quaternion get_station_quaternion(int station_id) = 0;
  int set_device_to_receive_saved_calibration();
  int set_device_for_calibration(void);
  void save_current_calibration_to_file(int station_id, int station_number);
  virtual int send_saved_calibration() = 0;
  bool calibrate_srv(polhemus_ros_driver::calibrate::Request &req, polhemus_ros_driver::calibrate::Response &res,
    std::string boresight_calibration_file);
  virtual bool calibrate(std::string boresight_calibration_file) = 0;
  libusb_device_handle *device_handle;
  int station_count;
  uint8_t endpoint_in;
  uint8_t endpoint_out;
  uint16_t endpoint_out_max_packet_size;
  uint16_t rx_buffer_size;
  uint16_t tx_buffer_size;
  int g_ntxcount;
  int g_nrxcount;
  std::vector<std::string> station_names_;
  std::string name;
  uint8_t sensors_right_glove, sensors_left_glove;
  uint8_t* g_txbuf;
  uint8_t* g_rxbuf;
  int latest_usb_read_result_;
};
#endif
