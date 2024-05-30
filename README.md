# ros2-simple-performance

Simple ROS 2 benchmark tool

## Build

```shell
mkdir -p ~/ros2_performance_ws/src && cd ~/ros2_performance_ws/src
git clone https://github.com/evshary/ros2-simple-performance.git
cd ../..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

