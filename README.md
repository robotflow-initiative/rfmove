# RFMove

# THIS LIBRARY IS STILL UNDER DEVELOPMENT
The first usable version is going to be released later in July 2022.

**Attention:** This repo use submodules. You can use `git clone --recurse-submodules <repo url>` to initialize submodules
automatically. If you download source code from releases, you need to fetch pybind11 manually from [pybind11 GitHub](https://github.com/pybind/pybind11)
and put it into `/extern/pybind11`.


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
sudo add-apt-repository ppa:savoury1/curl34
sudo apt-get update
sudo apt-get install curl

# cmake relay on curl so cmake would also be removed
sudo apt-get install cmake
```

Install pybullet (with python 3+). Note that you can  install pybullet in conda.
```bash
pip install pybullet
```


### Make soft links for libraries:
```bash
# start from repo root directory.
cd extern/lib  #All soft links can be removed
./rmlink
./mklink
```


### Make and Install
Source environment variables,if you want to use conda env,you need activate conda env at first.
```bash
# activate conda env
conda activate your_conda_env 

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

#### Run demo with rfuniverse
* download [RFUniverse_toBar.tar.xz](https://github.com/mvig-robotflow/rfmove/releases/tag/0.1.6)
```bash
tar -jxvf RFUniverse_toBor.tar.xz -C /your/rfu/path # uncompress Rfuniverse to your path
```

* modify your demo file
  You can find demo file about rfuniverse in "example/rfu".To run demo,you need modify your demo file as follows:   
```python
executable_file='/your/rfu/path/Player.x86_64'
scene_file='/your/rfu/path/Player_Data/StreamingAssets/SceneData/RFMoveTest.json'

# about rfmove 
rfmove_dir="/your/rfmove/path"

print("== Initalize Planner moveit model ==")
self.plannerspline=PlannerSpline("panda_arm")
self.plannerspline.init(rfmove_dir+"/resources/franka_convert.urdf",
                        rfmove_dir+"/resources/panda_arm_hand.srdf",
                        rfmove_dir+"/resources/kinematics.yaml",
                        rfmove_dir+"/resources/ompl_planning.yaml",
                        rfmove_dir+"/resources/joint_limits.yaml")
```

* make sure your urdf has correct modification
You can use urdf_converter to modification you urdf file, which introduced in "/notebook/QuickStart.ipynb",Or You can modify and replace urdf manually by "Ctrl+H" in vscode,as follows:
```
<collision>
      <geometry>
        <mesh filename="file:///home/ziye01/rfmove/resources/franka_description/meshes/collision/link0.stl"/>
      </geometry>
</collision>

####  from
<mesh filename="file:///home/ziye01/rfmove/resources/franka_description/meshes/collision/link0.stl"/>
####  to
<mesh filename="file://yourpath/your_robot_description/meshes/collision/link0.stl"/>
```

* Add you collision object 
```bash
cd  /your/rfu/path
./Player.x86_64 -edit  # open your rfuniverse env for add collision objects.
```
With the above command, you can add your own collider in the rfuniverse environment，Unfortunately, so far, rfuniverse only supports three types of collision objects, they are circle, box and capsule。

* run demo files
You can find demo about in "example/rfu"
1. test_rfmove_franka.py:run franka in rfuniverse
2. test_rfmove_toBar.py:run toBar in rfuniverse



## Build From Source With ROS-Kinetic and MoveIt-Kinetic

### Dependency
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- [Moveit 1 - Kinetic](http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/getting_started/getting_started.html)

We recommend installation through `apt install` directly.


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
- [x] Support Motion Plan.
- [x] Support IK-Fast kinematics solver.
- [ ] Make it pip installable.
- [x] Support ros controller.
