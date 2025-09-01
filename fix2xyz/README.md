# README for the fix2xyz Package

## Overview

The `fix2xyz` package provides functionality for transforming coordinates between different reference frames in a ROS2 environment. It includes nodes for converting fixed coordinates to XYZ coordinates and vice versa, as well as a test node for validating transformations.

## Package Structure

The package consists of the following key components:

- **Source Files**:
  - `src/fix_xyz_transform.cpp`: Implements the logic for coordinate transformations.
  - `src/fix2xyz_node.cpp`: Entry point for the fix2xyz node, initializing the ROS2 node and setting up necessary topics and services.
  - `src/fix2xyz_node_lib.cpp`: Provides library functions for the fix2xyz node.
  - `src/xyz2fix_node.cpp`: Entry point for the xyz2fix node, initializing the ROS2 node and setting up necessary topics and services.
  - `src/xyz2fix_node_lib.cpp`: Provides library functions for the xyz2fix node.
  - `src/tf_test.cpp`: Node for executing tests related to transformations.

- **Header Files**:
  - Located in the `include/fix2xyz` directory, containing declarations for the project's functions and classes.

- **CMake Configuration**:
  - `CMakeLists.txt`: Defines the build configuration for the ROS2 package.

- **Package Metadata**:
  - `package.xml`: Contains metadata about the package, including dependencies and maintainers.

## Dependencies

This package depends on several ROS2 packages and libraries, including:

- `rclcpp`
- `std_msgs`
- `nav_msgs`
- `sensor_msgs`
- `tf2`
- `tf2_ros`
- `geometry_msgs`
- `fix_msgs`
- `expo_crowd_msgs`
- `Eigen3`
- `PROJ`

## Building the Package

To build the `fix2xyz` package, use the following commands:

```bash
# Navigate to the workspace
cd ~/your_workspace/src

# Clone the repository (if applicable)
git clone <repository_url>

# Navigate to the workspace root
cd ~/your_workspace

# Build the package
colcon build --packages-select fix2xyz
```

## Running the Nodes

After building the package, you can run the nodes using the following commands:

```bash
# Source the workspace
source install/setup.bash

# Run the fix2xyz node
ros2 run fix2xyz fix2xyz_node

# Run the xyz2fix node
ros2 run fix2xyz xyz2fix_node

# Run the tf test node
ros2 run fix2xyz tf_test
```

## License

This package is licensed under the TODO license. Please refer to the license file for more details.

## Maintainer

- **Ubuntu** (ubuntu@todo.todo)

For any issues or contributions, please reach out to the maintainer.