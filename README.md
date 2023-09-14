# CI Statuses

Check | Status
---|---
Build|[<img src="https://codebuild.eu-west-2.amazonaws.com/badges?uuid=eyJlbmNyeXB0ZWREYXRhIjoiSDJUZUJtTVViUFZ4T3RrT3VWblVGS1IxMVR0U01pRU80RHhBcXBxd0RCUDlNd2paYW9xM2lWYktrUjRqRW93T0VGVHFtc29aa3E4bmZ2d1VISnp1QS93PSIsIml2UGFyYW1ldGVyU3BlYyI6IkVXVWlGemJWS2FycHJUS2QiLCJtYXRlcmlhbFNldFNlcmlhbCI6MX0%3D&branch=noetic-devel"/>](https://eu-west-2.console.aws.amazon.com/codesuite/codebuild/projects/auto_polhemus_ros_driver_noetic-devel_install_check/)
Style|[<img src="https://codebuild.eu-west-2.amazonaws.com/badges?uuid=eyJlbmNyeXB0ZWREYXRhIjoiNSs5d2M4Z0tjeE1tSUtKVE0waE5qZ0dQOFBGSGpnMUdnN0ZMSTRHSnhGL2N1RlNWTVhaWFlJU0wvVFlRbk9tVGREa3Q0NFpqdjl1RlJTZlB2bFZPQm1FPSIsIml2UGFyYW1ldGVyU3BlYyI6ImZsTEE2cHhhYTdIRHRlU3QiLCJtYXRlcmlhbFNldFNlcmlhbCI6MX0%3D&branch=noetic-devel"/>](https://eu-west-2.console.aws.amazon.com/codesuite/codebuild/projects/auto_polhemus_ros_driver_noetic-devel_style_check/)
Code Coverage|[<img src="https://codebuild.eu-west-2.amazonaws.com/badges?uuid=eyJlbmNyeXB0ZWREYXRhIjoidTRORnNlbFVQaVZiZDliYmFkTE5PSW9XcWFjVGxMaVRmQ0dtOExINlVJUDVKbjlGKzRhNGtEaXdJdS9oVlNoazRUdktubUFHTEcyNHQzeVRDeDZ0U3lVPSIsIml2UGFyYW1ldGVyU3BlYyI6InNNKzRYeTBreDhVMnpIZ0giLCJtYXRlcmlhbFNldFNlcmlhbCI6MX0%3D&branch=noetic-devel"/>](https://eu-west-2.console.aws.amazon.com/codesuite/codebuild/projects/auto_polhemus_ros_driver_noetic-devel_code_coverage/)

# Polhemus ROS Driver

This package publishes coordinate transforms for the Polhemus Liberty and Polhemus Viper sensors (stations) using the [`tf2` package](http://wiki.ros.org/tf2).

The user manual for the Polhemus Liberty can be found [here](http://polhemus.com/_assets/img/LIBERTY_User_Manual_URM03PH156-H.pdf). Currently there is no user manual for the Polhemus Viper system.

# Install 

Do not make changes to any files in `polhemus/etc`. Install requirements using:

```
$ cd ./etc
$ ./install.bash
```

Note that you will require the `sudo` password. 

# Usage

## Basic

1. `$ roscore`.
2. Open a new terminal.
3. `$ roslaunch polhemus_ros_driver start.launch`

The type of polhemus device can be selected by passing the parameter 'product_type', default value is liberty.
Currently this driver works with the Liberty and Viper systems.

## View frames

Open a new terminal. Start RVIZ (`$ rosrun rviz rviz`).

1. Global options: change *Fixed Frame* to `polhemus_base`. If there are two gloves connected and therefore two sources, they will be names `polhemus_base_0` and `polhemus_base_1`. Note that the Polhemus z-frame faces downwards and you need to publish a static transform to flip the orientation e.g. to polhemus_world (cf. example in launch).
2. Add -> *By display type* -> rviz -> TF.

# Launch file

The launch file `start.launch` allows you to change the zenith of the hemisphere and select the product type, the options are 'liberty' or 'viper'. The path and file name of a boresight calibration can also be passed to this launch file, the file does not have to contain calibration data but it must exist. The parameter 'hands' tells the driver whether it is dealing with a Shadow Glove unimanual or bimanual system, the options are 'left', 'right' or 'both'.

# Requirements

* ROS Melodic, see `package.xml`
* `libusb-1`
* `fxload` for Liberty only

# Driver description

The main loop is run in [polhemus_tf_broadcaster.cpp](src/polhemus_ros_driver/polhemus_tf_broadcaster.cpp), on startup the parameter for product_type is read from the parameter server, this would be either a viper or liberty device. The available USB devices are then searched and the driver detects if there are any Polhemus devices of that kind connected - if a device is found to be connected a new device object is then created.

- On connection, a reset command is sent to the Polhemus device, this is to guarantee that the device is not in continuous data streaming mode, the read buffer is then cleared.

- Boresight is then reset, this ensures that if there is boresight calibration data available, it is not layered onto a previous calibration as this command is accumulative if anything other than 0.

- Binary mode is activated, this action only applies to the Liberty system and sets all responses from the Liberty to be in Binary - the other option is ASCII.

- A query is made to find the number of active stations, a station is a sensor and source pair.

- The data type is set to 'quaternion'.

- Two services are then advertised, one to calibrate the sensors and another to select a source that a sensor should give it's position and orientation relative to. This is only an option for the Viper system as the Liberty has just one source per device.

- The hemisphere is then set based on the parameters passed in the launch file. Since the sensors can only operate in one hemisphere at a time relative to the source, it is necessary to tell the system which side of the source they will be on, for each station. We select the lower hemisphere.

- The data structure for the Liberty position and oriantation responses is generated, Viper responses are handled differently and do not require this. For Liberty a structure is built based on the number of connected stations - this means that after launch, more sensors cannot be connected.

- We set the data mode to continuous so that there is a constant stream of position and orientation data available. For the Liberty system this is sent at 240Hz, for Viper it can be up to 960Hz.

- The send saved calibration command checks the boresight calibration file which is passed in the launch file to see if there is any existing calibration data available, if there is, the sensors are calibrated.

The main loop is then started. This loop starts by reading a set of position and orientation data. A sensor count is returned. A sensor count of -1 is a failed command, 0 means no data was received. Both of these results point to a potential issue, so a device reset it completed. If data is received, the station count is updated - this allows the driver to deal with sensors dropping in and out.

The stations are then looped over and a stamped transform is populated with the received data. The Liberty system is only configured for one Shadow Glove to be connected. For Viper, half of the available sensor slots are alocated to a right glove and half to a left glove. Source 1 is automatically assigned to the first set of sensors and source 3 is assigned to the second set.  If there is a bimanual system connected, transforms will be published relative to two different frames.

# Communication protocols

The communication protocols for the Liberty and Viper systems are vastly different. All Liberty commands are sent as a string. Descriptions of all the available Liberty commands are in the [user manual](http://polhemus.com/_assets/img/LIBERTY_User_Manual_URM03PH156-H.pdf). Responses are only received from the Liberty if the command is directly requesting data. If a command were to fail, there is no way to determine this.

With the Viper system, the commands have a set format. Each command has a type, an associated action, 2 optional arguments and a config. There are additional sections of the commands such as a preamble, size and a CRC but a level of abstraction in the viper protocol means that these fields are filled automatically. The viper_protocol.h contains all of the options for these paramters in the form of enums and structs. The optional arguments aren't frequently used. When the first optional argument is used, it is usually set to the station id which the command is directed to, a -1 in this field meant all stations.

# Polhemus device available functions
Here is a description of all of the available functions in the [Polhemus device code](src/polhemus_ros_driver/polhemus.cpp).
- device_write, using the libusb package, commands are written and sent to the Polhemus device. Arguments are a pointer to a buffer of bytes to be sent, the number of bytes to be sent and a timeout value.
- device_init, initialise the usb device, the usb interface is configured and claimed.
- device_send, an abstraction to the device_write command, takes a pointer to the command and the count of bytes to be sent.
- device_read, read data from Polhemus device with the libusb library, arguments are a pointed to a buffer to receive the data, the number of bytes to receive and a boolean to set if a timeout results in an error.
- device_clear_input, reads bytes to clear the buffer.
- send_saved_calibration, if there is calibration data available in the boresight calibration file, send it to the Polhemus device, one station at a time. A reset boresight request is made, then one set of position and orientation data is read and continuous data mode is stopped. The saved calibration data for a station is read from the parameter server. The viper data mode is set to Euler so that both liberty and viper commands can be of the same format (Liberty sets boresight with euler values only). The quaternion from the position and orientation data that was just read is converted to roll pitch and yaw values in degrees. The values to be sent as a boresight command are then the current angles minus the values stored in the calibration file. The data type is then set back to quaternion and continuous data mode resumed - this completes the calibration from file request.
- calibrate, similar to the previous command, but the current roll pitch and yaw values are saved to file. The command sent to the Polhemus device is a set of 0's and can be sent to all stations at once.
- calibrate_srv,  this provides a service to call the calibrate function.
- persist_srv, this provides a service to set some settings to be persistent.
- device_reset, reset the device by stopping continuous mode and clearing the read buffer.
- request_num_of_stations - queries the polhemus device for the number of active stations, a station is a connected sensor and source pair.
- set_hemisphere, set or change the “hemisphere of operation” vector, arguments are x, y and z of the vector.
- define_data_type, select the format of the orientation data to be received, either euler or quaternion.
- device_binary_mode, the format of response from the Liberty can either be ASCII or binary, this sets the mode to be binary.
- generate_data_structure, only applies to the Liberty system, based on the station count received from request_num_of_stations, a structure is built to accomodate data from that number of sensors in a data receive.
- receive_pno_data_frame, calls device_read and extracts position and orientation data. Station count is updated.
- fill_pno_data, populated a stamped transform message with the most recent position and orientation data. Arguments are a pointer to a stamped transform and and index value which is the index position of the station in the received data structure, i.e. station count count be 3 but the sensor IDs could be non continuous. 
- device_data_mode, select the mode in whih data is received, a single data frame can be requested or data can be continuously received. A reset request turns off continuous mode. Argument is the mode type, potential modes are in the data_mode_e enum.
- set_boresight, this command causes the sensor to be electronically aligned in orientation (and
optionally, position) with the user system coordinates. Argument reset_origin will reset the position to 0. 4 arguments define the reference angles at the current orientation, these arguments are in zyx order, there is an optional 4th argument as for the viper system a quaternion can be sent. The command can be sent to a single station or all stations with the station ID set to -1.
- reset_boresight, resets any previous boresight request, this is called before any calibration request as the software could be re-launched without power cycling the device, this would result in the boresight angles accumulating and resulting in an incorrect calibration.
- persist_commands, a small subset of the viper configuration can be set to be persistent, those settings that can be made persistent will be made so by calling this function. Currently not used.
- set_source, for any sensor, the source which it's position and orientation is measured relative to can be set, this command required a source ID and sensor ID, all sensors can be selected with an ID of -1.
- get_quaternion, retrieves orientation data from the most recent frame received, returns a quaternion, used in the boresight calculations.
- receive_data_frame, Viper only, reads response from Viper system, called after sending all commands to check for errors. Argument is the type of command that a request is expected for, this assists with matching requests to responses, all options are in the viper_cmds_e enum.

# Debugging/testing
To mock the driver in order to test other programs (e.g. `knuckle_position_calibration_action_server.py`):
```bash
roslaunch polhemus_ros_driver polhemus_mock.launch testing_in_isolation:=true rosbag_path:=<your_rosbag_path>
roslaunch polhemus_ros_driver start.launch start_driver:=false
```
