# pointcloud_to_laserscan
This package provides a ROS2 node that converts PointCloud2 data to LaserScan data. The filtered points are projected to a 2D plane and then converted to a 1D LaserScan message.

# Usage

Launch the pointcloud_to_laserscan node:

```bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan
```

View the output LaserScan topic:

```bash
ros2 topic echo /output_laserscan
```


# Parameter

min_z (default: 0.0): Minimum z-axis value to be projected in 2D
max_z (default: 1.0): Maximum z-axis value to be projected in 2D