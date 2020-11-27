
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


#include <polhemus_ros_driver/liberty.hpp>

#ifdef DEBUG
#include <stdio.h>
#define warn(as...) { fprintf(stderr, "%s:%d: ", __FILE__, __LINE__); \
    fprintf(stderr, as); }
#else
#define warn(as...)
#endif


Liberty::Liberty(std::string name, uint16_t rx_buffer_size, uint16_t tx_buffer_size) : Polhemus(name, rx_buffer_size, tx_buffer_size)
{
}

Liberty::~Liberty(void) {}

/** this resets previous `c' commands and puts the device in binary mode
 *
 *  beware: the device can be misconfigured in other ways too, though this will
 *  usually work
 */

int Liberty::device_reset(void)
{
  // reset c, this may produce "invalid command" answers
  unsigned char command[] = "p";
  int size = sizeof(command) - 1;
  int retval = device_send(command, size);
  // remove everything from input
  device_clear_input();
  return retval;
}

int Liberty::device_binary_mode(void)
{
  unsigned char command[] = "f1\r";
  int size = sizeof(command) - 1;
  int retval = device_send(command, size);
  return retval;
}

void Liberty::generate_data_structure(void)
{
  stations = (liberty_pno_frame_t*) (malloc(sizeof(liberty_pno_frame_t) * station_count));
}

int Liberty::device_data_mode(data_mode_e mode)
{
  int size;
  int retval = RETURN_ERROR;
  switch (mode)
  {
    case DATA_CONTINUOUS:
    {
      unsigned char command[] = "c\r";
      size = sizeof(command) - 1;
      retval = device_send(command, size);
      return retval;
    }
    case DATA_SINGLE:
    {
      unsigned char command[] = "p";
      size = sizeof(command) - 1;
      retval = device_send(command, size);
      return retval;
    }
    default:
      return retval;
  }
}

int Liberty::receive_pno_data_frame(void)
{
  int retval = RETURN_ERROR;
  g_nrxcount = sizeof(liberty_pno_frame_t) * station_count;

  device_read(stations, g_nrxcount, true);
  if (g_nrxcount == 0)
  {
    return retval;
  }
  if (stations->head.init_cmd == LIBERTY_CONTINUOUS_PRINT_OUTPUT_CMD)
  {
    // update this as a sensor may have been dropped
    station_count = g_nrxcount / sizeof(liberty_pno_frame_t);
    retval = station_count;
  }
  return retval;
}

int Liberty::fill_pno_data(geometry_msgs::TransformStamped *transform, int &index)
{
  int retval = 0;

  // Set translation (conversion: inches -> meters)
  transform->child_frame_id = "polhemus_station_" + std::to_string(stations[index].head.station - 1);
  transform->transform.translation.x = 0.0254*stations[index].x;
  transform->transform.translation.y = 0.0254*stations[index].y;
  transform->transform.translation.z = 0.0254*stations[index].z;

  // Set rotation
  transform->transform.rotation.w = stations[index].quaternion[0];
  transform->transform.rotation.x = stations[index].quaternion[1];
  transform->transform.rotation.y = stations[index].quaternion[2];
  transform->transform.rotation.z = stations[index].quaternion[3];

  return retval;
}

int Liberty::define_data_type(data_type_e data_type)
{
  int retval = RETURN_ERROR;

  unsigned char* command;
  int size = 0;

  if (data_type == DATA_TYPE_QUAT)
  {
    unsigned char c[]  = "O*,8,9,11,3,7\r";  // quaternions

    command = c;
    size = sizeof(c) - 1;
    retval = device_send(command, size);
  }
  else if (data_type == DATA_TYPE_EULER)
  {
    unsigned char c[] = "O*,8,9,11,3,5\r";  // euler
    command = c;
    size = sizeof(c) - 1;
    retval = device_send(command, size);
  }
  else
  {
    return retval;
  }

  return retval;
}

int Liberty::request_num_of_stations(void)
{
  int retval = RETURN_ERROR;
  unsigned char command[] = { control('u'), '0', '\r', '\0' };
  active_station_state_response_t resp;
  int size = sizeof(command) - 1;
  int size_reply = sizeof(resp);
  device_send(command, size);

  device_read(&resp, size_reply, true);

  if (resp.head.init_cmd == LIBERTY_ACTIVE_STATION_STATE_CMD)
  {
    station_count = count_bits(resp.detected & resp.active);
    retval = 0;
  }
  return retval;
}

/* sets the zenith of the hemisphere in direction of vector (x, y, z) */
int Liberty::set_hemisphere(int x, int y, int z)
{
  int retval = RETURN_ERROR;
  int negative_sign_counter = 0;
  int initial_cmd_size = 10;

  if (x < 0)
    negative_sign_counter += 1;
  if (y < 0)
    negative_sign_counter += 1;
  if (z < 0)
    negative_sign_counter += 1;

  int buf_cmd_size = initial_cmd_size + negative_sign_counter;
  unsigned char command[buf_cmd_size];
  sprintf((char *)command, "h*,%d,%d,%d\r", x, y, z);

  int size = sizeof(command)-1;

  retval = device_send(command, size);

  return retval;
}

int Liberty::set_boresight(bool reset_origin, int station, float arg_1, float arg_2, float arg_3, float arg_4)
{
  int retval = RETURN_ERROR;
  unsigned char command[32];
  int size = sizeof(command);

  if (station == -1)
  {
    snprintf((char *)command, size, "b*,%d,%d,%d,%d\r", int(arg_1), int(arg_2), int(arg_3), reset_origin);
  }
  else
  {
    snprintf((char *)command, size, "b%d,%d,%d,%d,%d\r", station, int(arg_1), int(arg_2), int(arg_3), reset_origin);
  }

  size = 1;
  int i = 0;

  while (command[i] != 13)
  {
    size += 1;
    i++;
  }

  retval = device_send(command, size);
  return retval;
}

int Liberty::reset_boresight(void)
{
  int retval = RETURN_ERROR;
  unsigned char command[] = {control('b'), '*', '\r', '\0' };
  int size = sizeof(command) - 1;

  retval = device_send(command, size);
  return retval;
}

tf2::Quaternion Liberty::get_station_quaternion(int station_id)
{
  tf2::Quaternion q(stations[station_id].quaternion[1], stations[station_id].quaternion[2],
      stations[station_id].quaternion[3], stations[station_id].quaternion[0]);

  return q;
}
