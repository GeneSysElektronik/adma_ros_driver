# adma_ros2_driver

This is a ROS2 port of the original adma_ros_driver by the user lab176344 on github you can find [here](https://github.com/lab176344/adma_ros_driver).

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