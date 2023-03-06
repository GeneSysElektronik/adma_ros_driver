# ADMA ROS1 driver

Further Information can be found at the [GeneSys Technical Support Center](https://genesys-offenburg.de/support-center/). 

## Environment information

This setup was implemented and tested with the following conditions:
- Ubuntu 20.04
- ROS1 Noetic

Since it does not really use Noetic-specific code, it should also work with ROS1 Melodic on Ubuntu 18.04.

## Usage
1. Create workspace and clone this repository
```bash
mkdir -p ~/ros1_ws/src
cd ~/ros1_ws/src
git clone -b ros1_master $REPO_URL(HTPPS/SSH)
```

2. Build workspace
```bash
cd ~/ros1_ws
source /opt/ros/$INSTALLED_ROS_CONTRIB/setup.bash
catkin_make
```

3. Source workspace and launch
```bash
source ~/ros1_ws/devel/setup.bash
roslaunch adma_ros_driver ADMA_pub_Ethernet_new.launch
```

## Parameter configuration
For configuring the ADMA ROS Driver the according parameters in the `adma_ros_driver/launch/ADMA_pub_Ethernet_new.launch` file have to be modified.

| Parameter | Possible Values | Description|
|---|---|---|
| destination_ip | Any IP as string | IP adress of your ADMA |
| destination_port | any valid integer value | Destination Port of your ADMA |
| use_performance_check | True/False | True if you want to log informations about the performance |
| frame_id_X | any string name | ROS frame_id of the X topic |
| protocol_version | "v3.3.3" / "v3.3.4" | the ADMAnet protocol version of your ADMA |
| mode | "default" / "record" / "replay" | defines if you want to use it live with ADMA (default), record raw data received from ADMA (record) or replay the recorded raw data by replaying a rosbag (replay) |

## Supported ADMA Protocol versions
This driver node supports several protocol versions.
To switch between those, the`protocol_version` parameter in the `adma_ros_driver/launch/ADMA_pub_Ethernet_new.launch` file has to be modified.(e.g. define "v3.3.3"  to use version 3.3.3).
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

## Mode

Since V3.3.3 it is possible to record rosbags that also publish the raw byte data received from the ADMA by UDP. Therefore it subscribes to the topic `genesys/adma/data_recorded`. For recording the raw data the following steps have to be executed:

1. Switching the parameter `mode` in the `.launch`-file  to `record`.
2. Modify the parameter `rosbag_file` in the `.launch`-file to a valid destination folder where you want to save your recorded rosbag file.
3. start the launchfile with `roslaunch adma_ros_driver ADMA_pub_Ethernet_new.launch`

This will add the additional topic and also start the recording of the rosbag. It will ONLY record the raw data received from the ADMA by UDP, since the parsed data can be reproduced by replaying the rosbag (see section below). The process can be closed by pressing Ctrl+C after it's finished. 

### Replay recorded rosbag
For replaying the raw data the following parameters have to be configured:
1. switch the `mode` parameter to `replay`
2. modify the `rosbag_file` in the `.launch`-file to ensure it contains the correct path to your recorded rosbag (this time INCLUDING the file name, so e.g. `/home/$USER/records/my_new_record.bag`) 
3. start again with `roslaunch adma_ros_driver ADMA_pub_Ethernet_new.launch`.
 
## Remapping topics
If you want to modify the topics for some reason (e.g. make it compatible to your own ros nodes), you can modify the `<remap>...` entries of the `launch`-file. Here you just have to change the value of the `to=".."` attribute to let the driver publish to the desired topic.
