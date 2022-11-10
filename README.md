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