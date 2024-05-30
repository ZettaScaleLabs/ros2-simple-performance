# ros2-simple-performance

Simple ROS 2 benchmark tool

## Build

```shell
mkdir -p ~/ros2_performance_ws/src && cd ~/ros2_performance_ws/src
git clone https://github.com/evshary/ros2-simple-performance.git
cd ../..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

## Run

* Terminal 1: Run pong

```shell
ros2 run simple-performance pong
```

* Terminal 2: Run ping

```shell
ros2 run simple-performance ping
# Other configuration
ros2 run simple-performance ping --ros-args -p warmup:=15.0 -p size:=32
```
