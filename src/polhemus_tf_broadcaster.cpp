/*

  Communication library for a Polhemus Liberty v2 (tm) Motion tracker
  Copyright (C) 2008 Jonathan Kleinehellefort and Alexis Maldonado
  Intelligent Autonomous Systems Lab,
  Lehrstuhl fuer Informatik 9, Technische Universitaet Muenchen
  <kleinehe@cs.tum.edu>  <maldonad@cs.tum.edu>

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

/*
  This file is a modified version of polhemus_ros_node.cpp by Kleinehellefort and Maldonado.
  Modified by C. E. Mower, 2017.
*/

#include <libusb-1.0/libusb.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <signal.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <polhemus_ros_driver/liberty.hpp>
#include <polhemus_ros_driver/polhemus.hpp>
#include <polhemus_ros_driver/viper.hpp>
#include <polhemus_ros_driver/viper_protocol.h>
#include "polhemus_ros_driver/liberty_protocol.h"


/* Vendor 0x0f44 -> Polhemus */
#define VENDOR 0xf44

#define VPUSB_MAX_DISCOVERABLE 16

/* Product 0xff20 -> Liberty v2 Motion Tracker (with USB 2.0 using an EzUSB fx2
   chip)  after loading the firmware (it has ProductId 0xff21 before) */
#define LIBERTY_PRODUCT 0xff20
#define VIPER_PRODUCT 0xbf01


typedef struct _vp_usbdevinfo {
  int usbDevIndex;
  uint8_t usbBusNum;
  uint8_t usbDevNum;
  uint16_t usbVID;
  uint16_t usbPID;
  uint8_t usbNumInterfaces;
  uint8_t ep_in;
  uint8_t ep_out;
  uint16_t epout_maxPktsize;

  _vp_usbdevinfo() : usbDevIndex(-1), usbBusNum(0), usbDevNum(0), usbVID(0), usbPID(0), usbNumInterfaces(0), ep_in(0), ep_out(0), epout_maxPktsize(64) {};

}vp_usbdevinfo;

/* main loop running? */
static int go_on;

static void signal_handler(int s) {
  switch (s) {
  case SIGINT:
    go_on = 0;
    break;
  }
}

void release_usb(libusb_device_handle **usbhnd, vp_usbdevinfo &usbinfo)
{
  if ((usbhnd == 0) || (*usbhnd == 0))
    return;

  int r = 0;
  for (int i = 0; i < usbinfo.usbNumInterfaces; i++)
    r = libusb_release_interface(*usbhnd, i);

  libusb_close (*usbhnd);
  *usbhnd = 0;
  usbinfo = vp_usbdevinfo();

}

void find_endpoints(libusb_config_descriptor *conf_desc, int iface, uint8_t & ep_in, uint8_t & ep_out,
                    uint16_t & out_pktsize)
{
  for (int j = 0; j < conf_desc->interface[iface].num_altsetting; j++)
  {
    for (int k = 0; k < conf_desc->interface[iface].altsetting[j].bNumEndpoints; k++)
    {
      const struct libusb_endpoint_descriptor *p_ep;
      p_ep = &conf_desc->interface[iface].altsetting[j].endpoint[k];
      if ((p_ep->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK)
          & (LIBUSB_TRANSFER_TYPE_BULK | LIBUSB_TRANSFER_TYPE_INTERRUPT))
      {
        if (p_ep->bEndpointAddress & LIBUSB_ENDPOINT_IN)
        {
          if (!ep_in)
          {
            //printf("Ep in is false, endpoinst address is: %d", p_ep->bEndpointAddress);
            ep_in = p_ep->bEndpointAddress;
          }
        } else
        {
          if (!ep_out)
          {
            ep_out = p_ep->bEndpointAddress;
            out_pktsize = p_ep->wMaxPacketSize;
          }
        }
      }

    }
  }
}


int create_vip_list(libusb_context* pctx, libusb_device **&devlist, uint16_t vid, uint16_t pid,
                     vp_usbdevinfo arrDevInfo[], std::size_t &arrcount)
{
  int retval = RETURN_ERROR;
  ssize_t devcount = libusb_get_device_list(pctx, &devlist);
  if (devcount < 0)
    return (int) devcount; // returns error code < 0
  //struct libusb_device *found = NULL;
  //struct libusb_device_handle *found_dev_handle = NULL;
  libusb_device *dev;
  int iFoundCount = 0;

  int iFoundArrIndex = -1;
  int i = 0;
  while ((dev = devlist[i]) != NULL)
  {
    struct libusb_device_descriptor desc;
    retval = libusb_get_device_descriptor(dev, &desc);
    if (retval < 0)
      break;

    if (desc.idVendor == vid && desc.idProduct == pid)
    {
      iFoundCount++;
      iFoundArrIndex++;
      // populate the array if size permits
      if (iFoundCount <= (int) arrcount)
      {
        vp_usbdevinfo * p = &arrDevInfo[iFoundArrIndex];
        p->usbDevIndex = i;
        p->usbBusNum = libusb_get_bus_number(dev);
        p->usbDevNum = libusb_get_device_address(dev);
        p->usbVID = vid;
        p->usbPID = pid;
      }
    }
    i++;
  }

  arrcount = iFoundCount;

  return retval;
}

int discover_vip_pid(libusb_device_handle **usbhnd, vp_usbdevinfo &usbinfo, uint16_t vid, uint16_t pid)
{
  int retval = RETURN_ERROR;

  if (libusb_init (NULL))
    return retval;

  if (*usbhnd)
    release_usb(usbhnd, usbinfo);

  vp_usbdevinfo arrDevInfo[VPUSB_MAX_DISCOVERABLE];
  std::size_t arrcount = VPUSB_MAX_DISCOVERABLE;
  libusb_device **devlist;

  if ((retval = create_vip_list(NULL, devlist, vid, pid, arrDevInfo, arrcount)) < 0)
  {
    return retval;
  }

  if (arrcount == 0)
  {
    ROS_ERROR("[POLHEMUS] No Polhemus devices found. Please check that device is connected.\n");
    return RETURN_ERROR;
  }


  for (int d = 0; d < (int) arrcount; d++)
  {
    libusb_device * dev = devlist[arrDevInfo[d].usbDevIndex];
    libusb_device_handle * handle = 0;
    libusb_config_descriptor *conf_desc = 0;
    uint8_t nb_ifaces = 0, claimed_ifaces = 0;
    uint8_t ep_in = 0, ep_out = 0;
    uint16_t out_pktsize = 0;

    if ((retval = libusb_open(dev, &handle)))
    {
      ;
    }
    else if ((retval = libusb_get_config_descriptor(dev, 0, &conf_desc)))
    {
      libusb_close(handle);
    }
    else
    {
      nb_ifaces = conf_desc->bNumInterfaces;
      claimed_ifaces = 0;
      for (uint8_t i = 0; (i < nb_ifaces) && (retval == 0); i++)
      {
        if ((retval = libusb_claim_interface(handle, (int) i)))
        {
          ;
        }
        else
        {
          claimed_ifaces++;

          find_endpoints(conf_desc, (int) i, ep_in, ep_out, out_pktsize);
        }
      }

      if (claimed_ifaces == nb_ifaces)
      {
        *usbhnd = handle;
        usbinfo = arrDevInfo[d];
        usbinfo.usbNumInterfaces = nb_ifaces;
        usbinfo.ep_in = ep_in;
        usbinfo.ep_out = ep_out;
        usbinfo.epout_maxPktsize = out_pktsize;

        break;
      }
    }

    // clean up failed attempt before trying the next one.
    if (claimed_ifaces != nb_ifaces)
    {
      for (int k = 0; k < nb_ifaces; k++)
        libusb_release_interface(handle, k);
    }
    if (handle)
      libusb_close(handle);

    libusb_free_config_descriptor(conf_desc);
  }

  libusb_free_device_list(devlist, 1);

  if (!usbhnd)
    retval = -99;

  return retval;
}


int main(int argc, char** argv) {
  libusb_device_handle *g_usbhnd = 0;
  vp_usbdevinfo g_usbinfo;
  int i, nstations;
  double x_hs, y_hs, z_hs;
  struct timeval tv;
  uint16_t product_id;
  std::string product_type;
  std::string boresight_calibration_file;
  Polhemus *device;
  int retval = RETURN_ERROR;

  // Setup ros
  ros::init(argc, argv, "polhemus_tf_broadcaster");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.getParam("product_type", product_type);

  if (!private_nh.getParam("boresight_calibration_file", boresight_calibration_file))
  {
    ROS_ERROR("[POLHEMUS] Could not get boresight calibration file");
  }

  if (product_type == "liberty")
  {
    product_id = LIBERTY_PRODUCT;
    retval = discover_vip_pid(&g_usbhnd, g_usbinfo, VENDOR, product_id);
    if (retval == RETURN_ERROR)
    {
      //error connecting
      ROS_ERROR("[POLHEMUS] Error connecting to device.");
      return 0;
    }

    device = new Liberty(product_type, LIBERTY_RX_BUF_SIZE, LIBERTY_TX_BUF_SIZE);
    ROS_INFO("[POLHEMUS] Initialising liberty device.");
    device->endpoint_in = LIBERTY_ENDPOINT_IN;
    device->endpoint_out = LIBERTY_ENDPOINT_OUT;
  }
  else if (product_type == "viper")
  {
    product_id = VIPER_PRODUCT;
    retval = discover_vip_pid(&g_usbhnd, g_usbinfo, VENDOR, product_id);
    if (retval == RETURN_ERROR)
    {
      //error connecting
      ROS_ERROR("[POLHEMUS] Error connecting to device.\n");
      return 0;
    }

    device = new Viper(product_type, VIPER_RX_BUF_SIZE, VIPER_RX_BUF_SIZE);

    ROS_INFO("[POLHEMUS] Initialising Viper device.");
    device->endpoint_in = g_usbinfo.ep_in;
    device->endpoint_out = g_usbinfo.ep_out;
  }
  else
  {
    ROS_ERROR("[POLHEMUS] Could not find a valid Polhemus device type on parameter server.");
    abort();
  }

  device->nh = &nh;

  device->endpoint_out_max_packet_size = g_usbinfo.epout_maxPktsize;

  device->device_handle = g_usbhnd;

  retval = device->device_reset();
  if (retval == RETURN_ERROR)
  {
    ROS_ERROR("[POLHEMUS] Error resetting device.");
  }

  retval = device->reset_boresight();
  if (retval == RETURN_ERROR)
  {
    ROS_ERROR("[POLHEMUS] Error resetting boresight.");
    return 1;
  }

  device->device_binary_mode(); // activate binary mode

  retval = device->request_num_of_stations();
  if (retval == RETURN_ERROR)
  {
    ROS_ERROR("[POLHEMUS] Error reading number of stations.");
    return 1;
  }
  else
  {
    ROS_INFO("[POLHEMUS] Found %d stations.", device->station_count);
    nstations = device->station_count;
  }

  // define quaternion data type
  ROS_INFO("[POLHEMUS] Setting data type to quaternion");
  retval = device->define_data_type(DATA_TYPE_QUAT);

  if (retval == RETURN_ERROR)
  {
    ROS_ERROR("[POLHEMUS] Error setting data type.");
    return 1;
  }

  // Calibration service
  ros::ServiceServer service =
          private_nh.advertiseService<polhemus_ros_driver::calibrate::Request,
                                      polhemus_ros_driver::calibrate::Response>("calibration",
                                                                                boost::bind(&Polhemus::calibrate_srv,
                                                                                device, _1, _2,
                                                                                boresight_calibration_file));
  ROS_INFO("[POLHEMUS] Service ready to calibrate the sensors.");

  private_nh.getParam("x_hs", x_hs);
  private_nh.getParam("y_hs", y_hs);
  private_nh.getParam("z_hs", z_hs);

  ROS_INFO("[POLHEMUS] Setting the output hemisphere");
  retval = device->set_hemisphere(x_hs, y_hs, z_hs);
  if (retval == RETURN_ERROR)
  {
    ROS_ERROR("[POLHEMUS] Error setting hemisphere.");
    return 1;
  }

  device->generate_data_structure();

  /* set up signal handler to catch the interrupt signal */
  signal(SIGINT, signal_handler);

  go_on = 1;

  ROS_INFO("[POLHEMUS] Enabling continuous data mode...");
  retval = device->device_data_mode(DATA_CONTINUOUS);
  if (retval == RETURN_ERROR)
  {
    ROS_ERROR("[POLHEMUS] Error setting data mode to continuous.");
    return 1;
  }

  retval = device->send_saved_calibration();
  if (retval == RETURN_ERROR)
  {
    ROS_ERROR("[POLHEMUS] Calibration not loaded.");
  }

  gettimeofday(&tv, NULL);
  ROS_INFO("[POLHEMUS] Begin time: %d.%06d\n", (unsigned int) (tv.tv_sec), (unsigned int) (tv.tv_usec));

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  ros::Rate rate(240);

  int flag = 0;

  // Start main loop
  while(ros::ok()) {
    if (go_on == 0)
      break;

    // Update polhemus sensor count
    int sensor_count = device->receive_pno_data_frame();

    if (sensor_count == -1)
    {
      if (flag == 0 || flag == 2)
      {
        ROS_DEBUG("[POLHEMUS] No position and orientation data received from Polhemus system!!!");
        retval = device->device_reset();
        retval = device->device_data_mode(DATA_CONTINUOUS);
        flag = 1;
      }
    }
    else if (sensor_count == 0)
    {
      if (flag < 2)
      {
        ROS_WARN("[POLHEMUS] Polhemus system is reporting 0 sensors.");
        retval = device->device_reset();
        retval = device->device_data_mode(DATA_CONTINUOUS);
        flag = 2;
      }
    }
    else
    {
      if (flag >= 1)
      {
        ROS_DEBUG("[POLHEMUS] Position and orientation data now received from Polhemus system!!!");
        flag = 0;
      }
      /* Note: timestamp is the time in ms after the first read to the
         system after turning it on
         at 240Hz, the time between data sample is 1/240 = .00416_
         seconds.
         The framecount is a more exact way of finding out the time:
         timestamp = framecount*1000/240 (rounded down to an int)*
      */

      // Header info - acquired at same time = same timestamp
      transformStamped.header.stamp = ros::Time::now();
      transformStamped.header.frame_id = "polhemus_base";

      for (i=0; i < sensor_count; i++)
      {
        retval = device->fill_pno_data(&transformStamped, i);

        // Broadcast frame
        if (retval == 0)
        {
          br.sendTransform(transformStamped);
        }
      }
    }
    ros::spinOnce();
    rate.sleep();
  }

  retval = device->device_reset();

  if (retval == RETURN_ERROR)
  {
    ROS_ERROR("[POLHEMUS] Error resetting device.");
  }
  // // Shutdown
  libusb_close(g_usbhnd);
  delete device;

  return 0;
}
