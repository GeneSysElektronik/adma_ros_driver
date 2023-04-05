# adma_ros2_driver
Further Information can be found at the [GeneSys Technical Support Center](https://genesys-offenburg.de/support-center/). 

## Environment information
This setup was implemented and tested with the following conditions:
- Ubuntu 20.04
- ROS2 Galactic

Since it does not really use Galactic-specific code, it should also work with ROS2 Dashing/Foxy and Humble.

## Usage
1. Create workspace and clone this repository
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b ros_2 $REPO_URL(HTPPS/SSH)
```

2. Build workspace
```bash
cd ~/ros2_ws
. /opt/ros/$INSTALLED_ROS_CONTRIB/setup.bash
colcon build --symlink-install
```

3. Source workspace and launch
```bash
. install/setup.bash
ros2 launch adma_ros2_driver adma_driver.launch.py
```

## Parameter configuration
### Config File
For configuring the ADMA ROS Driver the according parameters in the `adma_ros2_driver/config/driver_config.yaml` file have to be modified.
If the workspace was built with `colcon build --symlink-install`, it is possible to restart the node after changing configuration parameters directly. Otherwise (built without '--symlink-install') it is necessary to rebuild the workspace to update the files. 
Same "linking" rule applies to the `launch.py` files. The available parameters are described in the table below.

| Parameter | Possible Values | Description|
|---|---|---|
| destination_ip | IP as String | IP address of your ADMA |
| destination_port | Port as Integer | Destination Port of your ADMA |
| use_performance_check | True / False | True if you want to log informations about the performance |
| gnss_frame | name as string | ROS frame_id of the NavSat topic |
| imu_frame | name as string | ROS frame_id of the IMU topic |
| protocol_version | "v3.2" / "v3.3.3" / "v3.3.4" | the ADMAnet protocol version of your ADMA |

### Launch File
#### Topic Remapping
It is possible to remap ROS topics in the driver to new namings by editing the `remappings=[..]` entries of the `launch.py`-file. 

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

### GeneSys Binary Raw Data (.gsdb)
The ADMA ROS driver is automatically saving the raw binary ADMA data as `*.gsdb` file in the directory of the executed terminal. The .gsdb data format is completly integrated in the GeneSys toolchain as shown in the following graphic.

![Integration of GSDB in the GeneSys Toolchain](https://user-images.githubusercontent.com/105273302/230074686-9b17826a-16d4-4f6a-83f7-8cd19daad914.jpg)

We recommend to always log .gsdb files for support purposes and for being able to reprocess the raw data with new ROS driver updates. If not needed, the logging can be disabled in the 
`adma_driver.launch.py` by setting `log_gsdb_arg` to False. 

### Rosbag 
For additionally recording a rosbag file, the `record_rosbag` argument in the `adma_driver.launch.py` have to be set to True. By adapting the list `recorded_topics`, the ROS topics getting written to the bag file can be chosen. 

![Online](https://user-images.githubusercontent.com/105273302/230114010-a0ac709a-d661-4f85-8706-973f8e96820c.jpg)

## Tools
Additional Tools can be found in the `adma_tools` folder and are separated based on their implementation language (`adma_tools_cpp`/ `adma_tools_py`).

### ROS2CSV-Converter
This tool subscribes to `/genesys/adma/data_scaled` and `/genesys/adma/status` and generates a CSV file for postprocessing. It can be used live or e.g. to convert a rosbag to CSV. 
The tool can be used by executing it in an additional terminal:

```bash
# ensure you have sourced the workspace also in this terminal
cd ~/ros2_ws
. install/setup.bash
ros2 run adma_tools_py ros2csv
```

This will create a `recorded_data.csv` in the destination where the `ros2 run adma_tools_py ros2csv` command is executed.
If an individual file naming needs to be used, the `adma_tools_py/adma_tools_py/ros2csv_converter.py` file can be adapted (self.filename = '').

![Convert CSV](https://user-images.githubusercontent.com/105273302/230113444-051bd497-82c2-4ae9-b0ee-10c916006a9a.jpg)


### BAG2GSDB-Converter
This tool subscribes to the `/genesys/adma/data_recorded` topic and generates a `*.gsdb`. By remapping it is also possible to subscribe to the `/genesys/adma/data_raw` topic. 

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
This value can be compared with the value that is defined in the `metadata.yaml` (`message_count` entry at the `/genesys/adma/data_recorded` topic). 

![Rosbag Replay](https://user-images.githubusercontent.com/105273302/230113644-8de6f0a6-aa27-4371-835d-bbb5a706c202.jpg)

### Replay recorded GSDB files
For reprocessing .gsdb files, the path to the desired .gsdb file has to be configured in the `gsdb_replay_config.yaml` at the parameter `gsdb_file`. After that, the `gsdb_replay.launch.py` tool can be launched.
The tool creates a .db3 file in the location where the tool was launched via terminal.  

```bash
ros2 launch adma_tools_cpp gsdb_replay.launch.py
```
![GSDB Replay](https://user-images.githubusercontent.com/105273302/230113700-73885377-8fa8-4456-91e6-786bbe50ad6b.jpg)
