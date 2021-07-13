/*

 Communication library for a Polhemus Liberty v2 (tm) Motion tracker
 Copyright (C) 2008 Jonathan Kleinehellefort <kleinehe@cs.tum.edu>
     Intelligent Autonomous Systems Lab,
     Lehrstuhl fuer Informatik 9, Technische Universitaet Muenchen

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


#ifndef LIBERTY_H
#define LIBERTY_H

#include <string>
#include <polhemus_ros_driver/polhemus.hpp>
#include "polhemus_ros_driver/liberty_protocol.h"

#define LIBERTY_ENDPOINT_IN 0x88
#define LIBERTY_ENDPOINT_OUT 0x4
#define LIBERTY_RX_BUF_SIZE  0x0200
#define LIBERTY_TX_BUF_SIZE  0x0200
#define LIBERTY_CONTINUOUS_PRINT_OUTPUT_CMD 67
#define LIBERTY_ACTIVE_STATION_STATE_CMD 21

class Liberty : public Polhemus
{
public:
    Liberty(std::string name, uint16_t rx_buffer_size, uint16_t tx_buffer_size);
    ~Liberty(void);
    void device_init();
    int request_num_of_stations(void);
    int device_reset(void);
    int device_binary_mode(void);
    int device_data_mode(data_mode_e mode);
    int receive_pno_data_frame(void);
    int fill_pno_data(geometry_msgs::TransformStamped *transform, int &index);
    void generate_data_structure(void);
    int define_data_type(data_type_e data_type);
    int set_hemisphere(int x, int y, int z);
    int set_boresight(bool reset_origin, int station, float arg_1, float arg_2, float arg_3, float arg_4 = 0);
    int reset_boresight(void);
    tf2::Quaternion get_station_quaternion(int station_id);
    bool calibrate(std::string boresight_calibration_file);
    int send_saved_calibration(int number_of_hands);
private:
    liberty_pno_frame_t *stations;
};
#endif
