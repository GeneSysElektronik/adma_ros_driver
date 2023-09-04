# adma_ros2_driver
Further Information can be found at the [GeneSys Technical Support Center](https://genesys-offenburg.de/support-center/). 

## Standard ROS Topics
### Integrated Topics
The ADMA uses a combination of GNSS-Receiver and different rate and acceleration sensors. Due to this, different ROS topics are getting filled with sensor, GNSS and combined measurement data as shown in the following list:

| Topic | Content | Description |
|---|---|---|
| /adma/data_scaled | ADMAnet v3.3.x | Standard ADMA data output format. |
| /adma/status | ADMAnet Status / Error / Warning v3.3.x | Standard ADMA Status, Error and Warning outputs. |
| /adma/heading | Heading as [std_msgs::Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html) | Heading in the standard ROS format. |
| /adma/velocity | Velocity as [std_msgs::Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html) | Velocity in the standard ROS format. |
| /adma/fix | [sensor_msgs::Navsatfix](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/NavSatFix.html) | GNSS Information in the standard ROS format. |
| /adma/imu | [sensor_msgs::imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html) | Inertial data in the standard ROS format. |
| /adma/data_raw | Raw UDP data stream | ADMA raw data as binary data stream. |
| /adma/odometrsy | [nav_msgs::Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) | Position, velocity and orientation |

### ROS Topic onfiguration
The ROS Topics can be output in desired measurement point locations in the vehicle. This can be done by using the ADMA POI's (Point of Interest). The POI's are defined in the ADMA Webinterface
through user defined offsets to the Measurement Reference Point (MRP). In the ADMA ROS Driver, the POI's in which each ROS topic shall be output can be selected with the relating ID in the 
Driver Config File (0 = MRP, 1-8 = POI 1-8). As Default, the ROS Topics are output in POI1. 

## Environment information
This setup was implemented and tested with the following conditions:
- Ubuntu 20.04
- ROS2 Galactic

Since it does not really use Galactic-specific code, it should also work with ROS2 Dashing/Foxy and Humble.
It does **NOT** work on Linux 16.04 in combination with ROS2 Ardent. 

## Usage
1. Create workspace and clone this repository
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b ros_2 $REPO_URL(HTPPS/SSH)
```

2. install all ROS dependencies (the warning for `ament_cmake_clange_format` can be ignored..)
```bash
cd ~/ros2_ws
# source ROS
. /opt/ros/$INSTALLED_ROS_CONTRIB/setup.bash
# initialize rosdep
sudo rosdep init # only required if not already done for other projects on your system
# update rosdep
rosdep update
# install dependencies
rosdep install --from-paths src --ignore-src -y
```

3. Build your workspace
```bash
cd ~/ros2_ws
. /opt/ros/$INSTALLED_ROS_CONTRIB/setup.bash
colcon build --symlink-install
```

4. Source workspace and launch
```bash
. install/setup.bash
ros2 launch adma_ros2_driver adma_driver.launch.py
```

## Parameter configuration
### Config File
For configuring the ADMA ROS Driver the according parameters in the `adma_ros2_driver/config/driver_config.yaml` file have to be modified.
If the workspace was built with `colcon build --symlink-install`, it is possible to restart the node after changing configuration parameters directly. Otherwise (built without '--symlink-install') it is necessary to rebuild the workspace to update the files. 
Same "linking" rule applies to the `launch.py` files. The available parameters are described in the table below.

### Launch File
#### Topic Remapping
It is possible to remap ROS topics in the driver to new namings by editing the `remappings=[..]` entries of the `launch.py`-file. 

#### Parameters
| Parameter | Possible Values | Description| Location |
|---|---|---|---|
| destination_ip | IP as String | IP address of your ADMA | driver_config.yaml |
| destination_port | Port as Integer | Destination Port of your ADMA | driver_config.yaml |
| use_performance_check | True / False | True if you want to log informations about the performance | driver_config.yaml |
| gnss_frame | name as string | ROS frame_id of the NavSat topic | driver_config.yaml |
| imu_frame | name as string | ROS frame_id of the IMU topic | driver_config.yaml |
| protocol_version | "v3.2" / "v3.3.3" / "v3.3.4" | the ADMAnet protocol version of your ADMA | driver_config.yaml |
| frame_ids | BOOL | Define the ROS topic names |  driver_config.yaml |
| log_gsdb | BOOL | Enable creating a GSDB file for logging the raw data |  adma_driver.launch.py |
| record_rosbag | BOOL | Enable logging a rosbag file |  adma_driver.launch.py |
| recorded_topics | ADMA topics as Strings | List of ADMA ROS Topics, that shall be logged into the rosbag |  adma_driver.launch.py |
| remappings | Topic names as Strings | List of ADMA topics and the names they shall get remapped to |  adma_driver.launch.py |

## Supported ADMA Protocol versions
This driver node supports several protocol versions.
To switch between those, the`protocol_version` parameter in the `config/driver_config.yaml` file has to be modified. (e.g. define "v3.3.4"  to use version 3.3.4. But care, this data format should also be configured in the ADMA Webinterface at menu 3 - Data / Ethernet Data Output Settings) .

* v3.2

        - UDP packet protocol of 768 bytes
        - supports 7 POI
- v3.3.3

        - UDP packet protocol of 856 bytes
        - supports 8 POI
- v3.3.4

        - UDP packet protocol of 856 bytes
        - supports 8 POI
        - only publishes scaled data and all relevant status/error/warning information
        - this also uses several standard-ROS msgs like Vector3 to represent XYZ coordinates instead of having 3 individual attributes
        - POI's are now integrated as a object list and can be accessed by their index 

## Data Output
The ADMA ROS driver is able to output two different file formats. 

![Online](https://github.com/GeneSysElektronik/adma_ros_driver/assets/105273302/f10a2ed1-1e7b-47b4-9f03-14a68fcdeac3)

### GeneSys Binary Raw Data (.gsdb)
![Integration of GSDB in the GeneSys Toolchain](https://user-images.githubusercontent.com/105273302/230074686-9b17826a-16d4-4f6a-83f7-8cd19daad914.jpg)

The ADMA ROS driver is automatically saving the raw binary ADMA data as `*.gsdb` file in the directory of the executed terminal. The .gsdb data format is completly integrated in the GeneSys toolchain as shown in the graphic above.

We recommend to always log .gsdb files for support purposes and for being able to reprocess the raw data with new ROS driver updates. If not needed, the logging can be disabled in the 
`adma_driver.launch.py` by setting `log_gsdb_arg` to False. 

### Rosbag 
For additionally recording a rosbag file, the `record_rosbag` argument in the `adma_driver.launch.py` have to be set to True. By adapting the list `recorded_topics`, the ROS topics getting written to the bag file can be chosen. 


## Tools
Additional Tools can be found in the `adma_tools` folder and are separated based on their implementation language (`adma_tools_cpp`/ `adma_tools_py`).

### ROS2CSV-Converter
![convert_csv](https://github.com/GeneSysElektronik/adma_ros_driver/assets/105273302/01339721-cd70-4363-9d6f-98b981377ab6)

This tool subscribes to `/genesys/adma/data_scaled` and `/genesys/adma/status` and generates a CSV file for postprocessing. It can be used live or e.g. to convert a rosbag to CSV. 
The tool can be used by executing it in an additional terminal:

#### Execution
```bash
# ensure you have sourced the workspace also in this terminal
cd ~/ros2_ws
. install/setup.bash
ros2 run adma_tools_py ros2csv
```

This will create a `recorded_data.csv` in the destination where the `ros2 run adma_tools_py ros2csv` command is executed.
The .csv file that gets created corresponds to the .gsda format specification of GeneSys. 
If an individual file naming needs to be used, the `adma_tools_py/adma_tools_py/ros2csv_converter.py` file can be adapted (self.filename = '').


### BAG2GSDB-Converter
![rosbag_replay](https://github.com/GeneSysElektronik/adma_ros_driver/assets/105273302/a8e36195-af1c-4f98-9e43-22b0da62cb17)

This tool subscribes to the `/genesys/adma/data_raw` topic and generates a `*.gsdb`. By remapping it is also possible to subscribe to the `/genesys/adma/data_recorded` topic (e.g. if you already have recorded rosbags with the deprecated `data_recorded` topic). 

NOTE: For converting ROS bags from ADMA ROS2 driver versions <= 2.1.3, the "adma/data_recorded" topic has to be remapped to "adma/data_raw". 

#### Parameters
| Parameter | Possible Values | Description | Location |
|---|---|---|---|
| rosbag_path | File location as String | Rosbag file to convert to GSDB | bag2gsdb.launch.py |
| replay_rate | Rate as Integer | Rate of data replay for increasing the replay speed. | bag2gsdb.launch.py |

#### Execution
```bash
# ensure you have sourced the workspace also in this terminal
cd ~/ros2_ws
. install/setup.bash
ros2 launch adma_tools_cpp bag2gsdb.launch.py
```
In the launch-file you have to define the path to the desired ros2 bag directory (the one where the `metadata.yaml` and `$FILE.db3` is located). The tool will create a `raw_data.gsdb` file inside the folder next to these files.
The launchfile will start both, `ros2 bag play` and the `converter tool` and stops everything when the replay of the bagfile is finished.
Additionally you can inject the `replay_rate` in the launchfile to speed-up the converting process.

NOTE: Increasing the replay_rate depends on the computer performance. It might lead to data losses. 
Therefore at the end of every process a log is output that contains the amount of received messages. 
This value can be compared with the value that is defined in the `metadata.yaml` (`message_count` entry at the `/genesys/adma/data_recorded` / `/genesys/adma/data_raw` topic). 


### Replay recorded GSDB files
![gsdb_replay](https://github.com/GeneSysElektronik/adma_ros_driver/assets/105273302/db434e24-3cc3-4241-ae61-a88be4c46539)

For reprocessing .gsdb files, the path to the desired .gsdb file has to be configured in the `gsdb_replay_config.yaml` at the parameter `gsdb_file`. After that, the `gsdb_replay.launch.py` tool can be launched.
The tool creates a .db3 file in the location where the tool was launched via terminal.  

#### Parameters
| Parameter | Possible Values | Description | Location |
|---|---|---|---|
| destination_ip | IP as String | Destination IP-address of replayed data stream | gsdb_replay_config.yaml |
| destination_port | Port as Integer | Destination Port of replayed data stream | gsdb_replay_config.yaml |
| destination_port | Port as Integer | Destination Port of replayed data stream | gsdb_replay_config.yaml |
| use_performance_check | BOOL | Enables time measurement for the process of receiving the UDP data until they get published | gsdb_replay_config.yaml |
| protocol_version | Version as String | version of the ADMANet data stream that is contained in the GSDB file | gsdb_replay_config.yaml |
| frame_ids | BOOL | Define the ROS topic names | gsdb_replay_config.yaml |
| ip_address | IP as String | IP-address of the gsdb_server | gsdb_replay_config.yaml |
| port | Port as Integer | Port of the gsdb_server | gsdb_replay_config.yaml |
| frequency | Frequency as Integer | Data frequency | gsdb_replay_config.yaml |
| protocol_version | Version as String | version of the ADMANet data stream that is contained in the GSDB file | gsdb_replay_config.yaml |
| gsdb_file | File location as String | GSDB Source file to replay | gsdb_replay_config.yaml |
| log_gsdb | BOOL | Enable creating a new GSDB file | gsdb_replay.launch.py |
| record_rosbag | BOOL | Enable logging a rosbag file | gsdb_replay.launch.py |

#### Execution
```bash
ros2 launch adma_tools_cpp gsdb_replay.launch.py
```


### Replay recorded GSDA files
![gsda_replay](https://github.com/GeneSysElektronik/adma_ros_driver/assets/105273302/13f3f3ff-5f80-478f-b3c9-0409e111b35b)

For reprocessing .gsda files, the path to the desired .gsda file has to be configured in the `gsda_replay_config.yaml` at the parameter `gsda_file`. After that, the `gsda_replay.launch.py` tool can be launched.
The tool creates a .db3 file in the location where the tool was launched via terminal.  

This tool was implemented for converting post processed data out of the ADMA PP software back to ROS2 data.

#### Parameters
| Parameter | Possible Values | Description | Location |
|---|---|---|---|
| frequency | Frequency as integer | Data frequency | gsda_replay_config.yaml |
| gsda_file | String | GSDA Source file to replay | gsda_replay_config.yaml |
| recorded_topics | Strings of ROS topics | Desired topics | gsda_replay.launch.py |

#### Execution
```bash
ros2 launch adma_tools_cpp gsda_replay.launch.py
```

