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

#ifndef POLHEMUS_ROS_DRIVER_LIBERTY_PROTOCOL_H
#define POLHEMUS_ROS_DRIVER_LIBERTY_PROTOCOL_H

typedef struct __attribute__((packed)) liberty_header_t
{
    char LY[2];
    unsigned char station;
    unsigned char init_cmd;
    unsigned char error;
    unsigned char reserved;
    uint16_t size;
}
liberty_header_t;

// this the default answer format for one station (O*,2,4,1) */
typedef struct __attribute__((packed)) liberty_default_pno_frame_t
{
    liberty_header_t head;
    float x;
    float y;
    float z;
    float azimuth;
    float elevation;
    float roll;
    char cr_lf[2];
}
liberty_default_pno_frame_t;

/* O*,8,9,11,3,7 */
typedef struct __attribute__((packed)) liberty_pno_frame_t
{
    liberty_header_t head;
    uint32_t timestamp;
    uint32_t framecount;
    int32_t distortion;
    float x;
    float y;
    float z;
    float quaternion[4];
}
liberty_pno_frame_t;

// O*,8,9,11,3,5
typedef struct __attribute__((packed)) liberty_euler_pno_frame_t
{
    liberty_header_t head;
    uint32_t timestamp;
    uint32_t framecount;
    int32_t distortion;
    float x;
    float y;
    float z;
    float az;
    float el;
    float ro;
}
liberty_euler_pno_frame_t;

typedef struct __attribute__((packed)) active_station_state_response_t
{
    liberty_header_t head;
    uint16_t detected;
    uint16_t active;
}
liberty_active_station_state_response_t;

#endif  // POLHEMUS_ROS_DRIVER_LIBERTY_PROTOCOL_H
