# fix_msgs Package

The `fix_msgs` package is designed for use with ROS2 and includes custom message definitions that facilitate the integration of various sensor data and orientation information.

## Message Definitions
  
- **FixWithOrientation**: This message defines the `FixWithOrientation` type, utilizing the ROS2 versions of `sensor_msgs/NavSatFix` and `geometry_msgs/Quaternion` to provide latitude, longitude, and orientation data.

## Dependencies

The package relies on the following ROS2 packages:
- `sensor_msgs`: For handling GPS fix messages.
- `geometry_msgs`: For handling quaternion orientation messages.

## Building the Package

To build the `fix_msgs` package, ensure that you have sourced your ROS2 environment and run the following commands in the root of your workspace:

```bash
colcon build
```

## Usage

After building the package, you can use the defined messages in your ROS2 nodes. Make sure to include the necessary headers for the messages you wish to use.

For example, to use `FixWithOrientation`, include the following in your node:

```cpp
#include "fix_msgs/msg/fix_with_orientation.hpp"
```

## License

This project is licensed under the MIT License. See the LICENSE file for more details.