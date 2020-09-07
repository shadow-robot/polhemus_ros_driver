/*
* Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#ifndef POLHEMUS_H
#define POLHEMUS_H

#include <ros/ros.h>
#include <ros/console.h>
#include <libusb-1.0/libusb.h>
#include <stdio.h>
#include <string>
#include <geometry_msgs/TransformStamped.h>
#include "polhemus_ros_driver/calibrate.h"
#include "polhemus_ros_driver/persist.h"
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <yaml.h>


#define INTERFACE 0
#define CONFIGURATION 1
#define TIMEOUT 1000
#define CRC_BYTES 4
#define SEUID 0
#define VPUSB_WRITE_TIMEOUT_MS 50
#define VPUSB_READ_TIMEOUT_MS 50
#define PI 3.14159265359

#define RETURN_ERROR -1

/* make control character out of ordinary character */
#define control(c) ((c) & 0x1f)

typedef enum data_mode_e {
    DATA_CONTINUOUS,
    DATA_SINGLE,
    DATA_RESET
} data_mode_e;

typedef enum data_type_e {
    DATA_TYPE_QUAT,
    DATA_TYPE_EULER
} data_type_e;

class Polhemus
{
public:
  Polhemus(std::string name, uint16_t rx_buffer_size, uint16_t tx_buffer_size);
  virtual ~Polhemus(void);
  int count_bits(uint16_t v);
  ros::NodeHandle *nh;

  /* set up usb interface and configuration, send initial magic and reset */
  int device_init(void);
  virtual int device_binary_mode(void);
  virtual int device_data_mode(data_mode_e mode);
  /* send a command */
  int device_send(uint8_t *cmd, int &count);
  /* read until the device doesn't send anything else */
  void device_clear_input(void);
  virtual int receive_pno_data_frame(void);
  virtual int fill_pno_data(geometry_msgs::TransformStamped *transform, int index);

  int device_read(void *pbuf, int &count, bool bTOisErr/*=false*/);
  int device_write(uint8_t *buf, int size, int timeout);
  virtual int device_reset(void);
  virtual int define_data_type(data_type_e data_type);
  virtual void generate_data_structure(void);
  virtual int set_hemisphere(int x, int y, int z);
  virtual int request_num_of_stations(void);
  virtual int set_boresight(bool reset_origin, int station, float arg_1, float arg_2, float arg_3, float arg_4 = 0);
  virtual int reset_boresight(void);
  virtual tf2::Quaternion get_quaternion(int index);
  virtual int set_source(int source);
  int send_saved_calibration(void);
  bool calibrate_srv(polhemus_ros_driver::calibrate::Request &req, polhemus_ros_driver::calibrate::Response &res, std::string boresight_calibration_file);
  bool persist_srv(polhemus_ros_driver::persist::Request &req, polhemus_ros_driver::persist::Response &res);
  bool calibrate(std::string boresight_calibration_file);
  virtual bool persist_commands(void);
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
  uint8_t* g_txbuf;
  uint8_t* g_rxbuf;
};
#endif
