# adma_ros2_driver

This is a ROS2 port of the original adma_ros_driver by the user lab176344 on github you can find [here](https://github.com/lab176344/adma_ros_driver).

It is implemented more "ros-like" and also supports several ADMA protocol versions (see below).

## Environment information
This setup was implemented and tested with the following conditions:
- Ubuntu 20.04
- ROS2 Galactic

Since it does not really use Galactic-specific code, it should also work with ROS2 Foxy or even Dashing.

## Usage
1. Create workspace and clone this repository
```bash
mkdir -p ~/ros2_ws/src
git clone -b ros_2 $REPO_URL(HTPPS/SSH)
```

2. build workspace
```bash
cd ~/ros2_ws
colcon build --symlink-install
```

3. source workspace and launch
```bash
. install/setup.bash
ros2 launch adma_ros2_driver adma_driver.launch.py
```

## Parameter configuration
To change the parameters, just modify the possible values in the `adma_ros2_driver/config/driver_config.yaml`. If you built the workspace with `colcon build --symlink-install` you can direct restart the node with the changed parameters, otherwise (built without `--symlink-install`) you need to rebuild the workspace to update the files. Same "linking" rule applies to the `launch.py` files.

## Supported ADMA Protocol versions
This driver node supports several protocol versions.
To switch between those, just change the value of the `protocol_version` settings in the `config/driver_config.yaml` file (e.g. define "v3.3.3" to use version 3.3.3).
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
For easily testing the parsing part without connecting to a real ADMA device,
it also contains a `data_server` that sends a fixed version-based UDP packet.

To work with it, simply use:
3. source workspace and launch
```bash
. install/setup.bash
ros2 launch adma_ros2_driver adma_driver_debugging.launch.py
# for changing config, you can modify the separate 'driver_config_debug.yaml' file
```

## Tools
This repository also contains some useful tools to work with the ADMA.
These can be found in the `adma_tools` package.

### ROS2CSV-Converter
This tool subscribes to `/genesys/adma/data_scaled` and `/genesys/adma/status`and generates a CSV file for postprocessing. It can be used live or e.g. to convert a rosbag to CSV. You just need to execute it in an additional terminal:

```bash
# ensure you have sourced the workspace also in this terminal
cd ~/ros2_ws
. install/setup.bash
ros2 run adma_tools ros2csv
```

This will create a `recorded_data.csv` in the destination where you execute the `ros2 run..` command.
If you want to use a different name, you can modify the code of `adma_tools/adma_tools/ros2csv_converter.py` line 11 (self.filename = '').
## Re-use old recorded data

Since V3.3.4 you can record rosbags that also publish the raw byte data received from the ADMA by UDP. Therefore it subscribes to the topic `genesys/adma/data_recorded`. To achieve this, you need to do the following steps:

```bash
# 1. record the rosbag
ros2 bag record -a
# 2. after recording, stop it and start a new recording with the desired topic
ros2 bag record /genesys/adma/data_recorded 
# 3. replay rosbag with remapped topic
ros2 bag play $ROSBAG_FOLDER --remap /genesys/adma/data_raw:=/genesys/adma/data_recorded 
```
After these steps you should have a new smaller rosbag that only contains the raw data in the desired topic.

Now you can prepare the configuration and launchfile:
1. switch the parameter `use_recorded_data` of the `config/driver_config.yaml` to `True`
2. modify the `rosbag_file_arg` in `launch/adma_driver_recorded_data.launch.py` to ensure it contains the correct path to your recorded rosbag

```bash
ros2 launch adma_ros2_driver adma_driver_recorded_data.launch.py
```