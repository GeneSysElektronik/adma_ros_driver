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
For configuring the ADMA ROS Driver the according parameters in the `adma_ros2_driver/config/driver_config.yaml` file have to be modified.
If the workspace was built with `colcon build --symlink-install`, it is possible to restart the node after changing configuration parameters directly. Otherwise (built without '--symlink-install') it is necessary to rebuild the workspace to update the files. 
Same "linking" rule applies to the `launch.py` files. The available parameters are described in the table below.

| Parameter | Possible Values | Description|
|---|---|---|
| destination_ip | Any IP as string | IP adress of your ADMA |
| destination_port | any valid integer value | Destination Port of your ADMA |
| use_performance_check | True/False | True if you want to log informations about the performance |
| gnss_frame | any string name | ROS frame_id of the NavSat topic |
| imu_frame | any string name | ROS frame_id of the IMU topic |
| protocol_version | "v3.2" / "v3.3.3" / "v3.3.4" | the ADMAnet protocol version of your ADMA |
| mode | "default" / "record" / "replay" | defines if you want to use it live with ADMA (default), record raw data received from ADMA (record) or replay the recorded raw data by replaying a rosbag (replay) |

## Supported ADMA Protocol versions
This driver node supports several protocol versions.
To switch between those, the`protocol_version` parameter in the `config/driver_config.yaml` file has to be modified.(e.g. define "v3.3.3"  to use version 3.3.3) .
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
        - additionally publishes the raw data (HEX) as separate String topic (could be recorded e.g. for post-processing)
        - this also uses several standard-ROS msgs like Vector3 to represent XYZ coordinates instead of having 3 individual attributes
        - POI's are now integrated as a object list and can be accessed by their index 

## Debugging
It is possible to test the ROS Driver without connecting to a real ADMA device by
using the integrated 'data_server' to send a fixed version-based UDP packet.

For using the server, refer to:
3. source workspace and launch
```bash
. install/setup.bash
ros2 launch adma_ros2_driver adma_driver_debugging.launch.py
# for changing config, you can modify the separate 'driver_config_debug.yaml' file
```

## Tools
This repository also contains some useful tools to work with the ADMA.
These can be found in the `adma_tools` folder and are in separated packages based on their implementation language
(`adma_tools_cpp`/ `adma_tools_py`).

### ROS2CSV-Converter
This tool subscribes to `/genesys/adma/data_scaled` and `/genesys/adma/status` and generates a CSV file for postprocessing. It can be used live or e.g. to convert a rosbag to CSV. 
The tool can be used by executing it in an additional terminal:

```bash
# ensure you have sourced the workspace also in this terminal
cd ~/ros2_ws
. install/setup.bash
ros2 run adma_tools_py ros2csv
```

This will create a `recorded_data.csv` in the destination where the `ros2 run..` command is executed.
If an individual file naming needs to be used, the `adma_tools_py/adma_tools_py/ros2csv_converter.py` file can be adapted in line 11 (self.filename = '').


### BAG2GSDB-Converter
This tool subscribes to the `/genesys/adma/data_recorded` topic and generates a `*.gsdb` file that can be used for the ADMA-PostProcessing application.

```bash
# ensure you have sourced the workspace also in this terminal
cd ~/ros2_ws
. install/setup.bash
ros2 launch adma_tools_cpp bag2gsdb.launch.py
```
In the launch-file you have to define the path to the desired ros2 bag folder (the one where the `metadata.yaml` and `$FILE.db3` is located). The tool will create the `raw_data.gsdb` file inside the folder next to these files.
The launchfile will start both, ros2 bag play and the converter tool and stops everything when the replay of the bagfile is done.
Additionally you can inject the `replay_rate` in the launchfile to speed-up the converting process.

NOTE: Increasing the replay_rate depends on the computer performance so it may can lead to missing messages. Therefore the converter prints at the end a log to the console how many messages where written to the defined file. This value you can compare with the value that is defined in the `metadata.yaml` (look for the topic `/genesys/adma/data_recorded` and then the `message_count` entry). All messages where written when those values are the same (1 less is also ok..). 

## Re-use old recorded data

Since V3.3.4 it is possible to record rosbags that also publish the raw byte data received from the ADMA by UDP. Therefore it subscribes to the topic `genesys/adma/data_recorded`. For recording the raw data the following steps have to be executed:

1. Switching the parameter `mode` to `record`.

This will add the additional topic and also start the recording of the rosbag. The process can be closed by pressing Ctrl+C after it's finished. 

### Replay recorded rosbag
For replaying the raw data the following parameters have to be configured:
1. switch the `mode` parameter to `replay`
2. modify the `rosbag_file_arg` in `launch/adma_driver.launch.py` to ensure it contains the correct path to your recorded rosbag
3. start again with `ros2 launch adma_ros2_driver adma_driver.launch.py`.

### Remapping of ROS topics
If you want to modify the topics for some reason (e.g. make it compatible to your own ros nodes), you can modify the `remappings=[..]` entries of the `launch.py`-file. Here you just have to change the value of the right string to let the driver publish to the desired topic.