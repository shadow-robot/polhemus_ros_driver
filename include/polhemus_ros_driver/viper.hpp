/*
* Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#ifndef VIPER_H
#define VIPER_H

#include <string>
#include <polhemus_ros_driver/polhemus.hpp>
#include <polhemus_ros_driver/viper_protocol.h>


#define VIPER_ENDPOINT_IN 0x81
#define VIPER_ENDPOINT_OUT 0x02
#define VIPER_TX_BUF_SIZE  0x400
#define VIPER_RX_BUF_SIZE  0x400

class Viper: public Polhemus
{
public:
  Viper(std::string name, uint16_t rx_buffer_size, uint16_t tx_buffer_size);
  ~Viper(void);
  void device_init();
  int request_num_of_stations(void);
  int device_reset(void);
  int device_data_mode(data_mode_e mode);
  int receive_pno_data_frame(void);
  int fill_pno_data(geometry_msgs::TransformStamped *transform, int &index);
  int define_data_type(data_type_e data_type);
  int set_hemisphere(int x, int y, int z);
  bool src_select_srv(polhemus_ros_driver::set_source::Request &req, polhemus_ros_driver::set_source::Response &res);
  ros::ServiceServer source_select_service;
private:
  uint32_t calc_crc_bytes(uint8_t *data, uint32_t count);
  void crc_16(uint32_t * crc, uint32_t data);
  viper_full_header_t* fill_command(uint32_t cmd, uint32_t act, uint32_t arg1 = 0, uint32_t arg2 = 0, void *payload = 0,
                                    uint32_t payload_size = 0);
  void prepare_frame(uint8_t buffer[], int &txbytes);
  int receive_data_frame(viper_cmds_e cmd_type);
  int set_boresight(bool reset_origin, int station, float arg_1, float arg_2, float arg_3, float arg_4 = 0);
  int reset_boresight(void);
  tf2::Quaternion get_station_quaternion(int station_id);
  int set_source(int source, int station_id);
  bool calibrate(std::string boresight_calibration_file);
  int send_saved_calibration(int number_of_hands);
  bool persist_commands(void);
  bool persist_srv(polhemus_ros_driver::persist::Request &req, polhemus_ros_driver::persist::Response &res);
  CVPSeuPno pno;
  uint32_t sensor_map;
};
#endif
