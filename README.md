# Implement NMPC for mobile robot using ROS and ACADO library

## Installation
Install ACADO toolkit follow this [link](https://acado.github.io/install_linux.html)

## Compiling
```
# 0. Download codes
$ cd catkin_ws/src
$ git clone https://github.com/duynamrcv/mpc_ros/
# 1. Code generation
$ cd mpc_export_code/
$ mkdir build && cd build
$ cmake ..
$ make
$ ./mpc

# 2. Build ACADO static library
$ mv simple_mpc_export/* ../../acado_mpc_export/
$ cd ../../acado_mpc_export
$ make 

# 3. catkin make
$ cd ~/catkin_ws/
$ catkin_make
```

## Demo
![](results/demo_mpc.mp4)