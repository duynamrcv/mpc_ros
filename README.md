# Implement NMPC for mobile robot using ROS and ACADO library

## Installation
### ACADO
Install ACADO toolkit follow this [link](https://acado.github.io/install_linux.html) or as follows:
```
# 1. Install dependences
$ sudo apt-get install gcc g++ cmake git gnuplot doxygen graphviz
# 2. Clone ACADO library
$ git clone https://github.com/acado/acado.git -b stable ACADOtoolkit
# 3. Build
$ cd ACADOtoolkit
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
# 4. Add source to .bashrc
$ echo "source ACADOtoolkit/build/acado_env.sh" >> ~/.bashrc
```

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