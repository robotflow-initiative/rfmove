# RFMove

# THIS LIBRARY IS STILL UNDER DEVELOPMENT
The first usable version is going to be released later in June 2021.

**Attention:** This repo use submodules. You can use `git clone --recurse-submodules <repo url>` to initialize submodules
automatically. If you download source code from releases, you need to fetch pybind11 manually from [pybind11 GitHub](https://github.com/pybind/pybind11)
and put it into `/extern/pybind11`.

## Install Binary
Follow introduction in `<release>/install.md`.

## Build with Standalone Libraries
This library use and wrap the shared libraries from ROS and MoveIt.
If you want to build this library without the installation of ROS or MoveIt,
you need to provide all required shared libraries and header files standalone.

### Install Dependency
Install Eigen3 which is the geometry mathematics lib used by MoveIt. Eigen3 in
cpp is somewhat like NumPy in Python.
```bash
sudo apt install libeigen3-dev
```

Install gcc. Of course we need gcc.
```bash
sudo apt install build-essential
```

Install boost. Boost is widely used ... nearly everywhere.
```bash
sudo apt install libboost-all-dev
```
However, ros kinetic use boost 1.58.0 which is no longer available in some new system version.
In that case, we packed all boost 1.58.0 libraries standalone.

<!--
Install urdfdom headers. 
Download it from [urdfdom_headers github](https://github.com/ros/urdfdom_headers/tree/master).
Note that moveit-kinetic use urdfdom version 0.4. 
```bash
cd urdfdom_headers-1.0.5
mkdir build
cd build
cmake ..
sudo make install
```
-->

Install [TinyXML](http://www.grinninglizard.com/tinyxml/) which is the xml parser used by ros.
However, there is a new TinyXML-2 available but ros still use TinyXML.
TinyXML only use `make` as its build system and does not provide binary release.
In that case, it would be somewhat difficult to install it. Catkin provide a package
to install it. Here, we repackage TinyXML as a cmake project in `extern/tinyxml`.
The source code is downloaded from [SourceForge](https://sourceforge.net/projects/tinyxml).
You can build and install TinyXML into `/use/local` through:
```bash
cd extern/tinyxml
mkdir build
cmake ..
sudo make install
```
You should find `libtinyxml.so; libtinyxml.so.2.6.2` in `/usr/local/lib` and `tinyxml.h` in `/usr/local/include`.

If you build this lib on ubuntu 18.04+, you should reinstall `libcurl4` as the curl from default software repo does
not support SSLv3.
```bash
dpkg --list | grep curl
sudo apt --purge remove libcurl4
sudo add-apt-repository ppa:xapienz/curl34
sudo apt-get update
sudo apt-get install curl

# cmake relay on curl so cmake would also be removed
sudo apt-get install cmake
```

Install pybullet (with python 2.7). Note that conda can not install pybullet for python2.7.
```bash
pip install pybullet
```
(Maybe it's time for us to use python3)

### Download Libraries and Includes
Put directory `include` and `lib` into `extern`.

Make soft links for libraries:
```bash
# start from repo root directory.
cd extern/lib
./mklink
```
All soft links can be removed by `./rmlink`.

### Make and Install
Source environment variables
```bash
source set_env.bash
```
That would set the header files and libraries search path.

Build
```bash
mkdir install
mkdir build
cd build
cmake ..
make install
```
All shared libraries, public header files, and executable file would be installed into `install` folder (so that
we do not need administrator privilege).

### Run demo
```bash
cd src
python pybullet_demo.py
```

## Build From Source With ROS-Kinetic and MoveIt-Kinetic

### Dependency
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- [Moveit 1 - Kinetic](http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/getting_started/getting_started.html)

We recommend installation through `apt install` directly.

### Build and Make
```bash
mkdir install
mkdir build
cd build
cmake ..
make install
```

All libraries, resources, bindings, and includes should be built and installed in `install`.

### Run demo
Source the environment of ros kinetic through its `setup.bash` file so that we can find its shared libraries.
```bash
source /opt/ros/kinetic/setup.bash
```

Append installed lib path to `LD_LIBRARY_PATH` so that we can find the shared library we built.
```bash
export LD_LIBRARY_PATH=${THIS_REPO_PATH}/install/lib
```

Pybullet demo (Need pybullet)
```bash
# Python2 Only For Now!
python demo.py
```

executable demo
```bash
cd install bin
LD_LIBRARY_PATH=<repo>/install/lib:$LD_LIBRARY_PATH ./moveit_not_ros_demo
```

## Source Tree
- **CMakeLists.txt**: Top level build script. Declare all dependencies and targets.
- **src/**: Source directory
    - **robot_model/**: Helper and Wrapper classes about robot model and kinematics solver.
        - **CMakeLists.txt**: Build script for shared library `librobot_model_noros.so`
        - **src/**
        - **include/robot_model/**
    - **python/**: Source of PyMove binding code.
        - **CMakeList.txt**: Build script for python module `moveit_noros.so`
        - **RobotModelLoaderPy.cpp**: Core binding code.
        - **include/**: Split binding code.
    - **util/**: Some common utils.
    - **model_and_state_demo.cpp**: Implement of MoveIt robot model and robot state tutorial without ros.
    - **model_and_state_demo.py**: Implement of MoveIt robot model and robot state tutorial using PyMove.
- **extern/**: Extern source directory
    - **kdl_kinematics_plugin/**: Implementation of kdl kinematics solver without ros.
        - **CMakeLists.txt**: Build script for shared library `libkdl_kinematics_noros.so`
        - **src/**
        - **include/kdl_kinematics_plugin/**: Only `kdl_kinematics_plugin.h` is installed.
    - **pybind11/**: Header-only library pybind11.
- **resources/**: Resource file copyed from `panda_moveit_config` package.

## TODO
- [ ] Support Motion Plan.
- [ ] Support IK-Fast kinematics solver.
- [ ] Make it pip installable.
- [ ] Support ros controller.
