# 安装
* 需要安装pyrfuniverse,详情请见[pyrfuniverse-github](https://github.com/mvig-robotflow/pyrfuniverse)
* 需要安装rfmove,在安装过程中需要注意以下步骤：
  *  首先在安装pyrfuniverse过程中，需要安装conda rfuniverse  py=3.8,如果你选择了这个环境，那么rfmove的安装必须在相同的环境下，否则你的安装的python会有不同的版本
  *  重新连接库，在extern/lib/中，运行以下脚本
```bash
./rmlink   # 删除原本的库
./mklink   # 创建新的连接
```
  * 确保你的curl库的正确
```bash
dpkg --list | grep curl
sudo apt --purge remove libcurl4
sudo add-apt-repository ppa:xapienz/curl34
(sudo add-apt-repository ppa:savoury1/curl34)  //if you can also use this repository at ubuntu20.04 
sudo apt-get update
sudo apt-get install curl

# cmake relay on curl so cmake would also be removed
sudo apt-get install cmake
```
  * 正式安装
```bash
mkdir install
mkdir build
cd build
cmake ..
make install
```

# 运行
1. 下载Universe-Player,可在rfmove创库0.1.6版本的中找到下载
```bash
# 通过以下命令可以变价rfuniverse的世界环境，
./Player.x86_64 -edit
# 如果你的运动规划在运动过程中老是碰撞地面，可以在你的机器人底部添加一个box，box的中心坐标在几何中，沉入地面就可以了
# 使用以下命令添加地面碰撞物体
move2.detectUnityObject(); #本命令会在运动规划中，将所有universe环境中的长方体，球形以及胶囊体加入运动规划的避障过程

```
2. 修改脚本的以下内容test_rfmove_franka.py的以下内容
```python
# 修改以下内容，为你的Universe-Player的绝对路径
executable_file='/home/ziye01/Player/RFUniverse_Player_For_Linux_v0.1.2b/RFUniverse/Player.x86_64',
scene_file='/home/ziye01/Player/RFUniverse_Player_For_Linux_v0.1.2b/RFUniverse/Player_Data/StreamingAssets/SceneData/RFMoveTest.json',
# 修改以下内容，为你的rfmove的绝对路径
rfmove_dir="/home/ziye01/rfmove"
```
3. 确保你urdf包已经被正确修正(非常重要，否则碰撞会无效)
使用urdf_converter，确保你的urdf文件已经被修改，可以在notebook中找到对应说明文档，也可以对urdf文档直接修改，
```python
# 我们使用的是franka_convert.urdf",

print("== Initalize Planner moveit model ==")
self.plannerspline=PlannerSpline("panda_arm")
self.plannerspline.init(rfmove_dir+"/resources/franka_convert.urdf",
                        rfmove_dir+"/resources/panda_arm_hand.srdf",
                        rfmove_dir+"/resources/kinematics.yaml",
                        rfmove_dir+"/resources/ompl_planning.yaml",
                        rfmove_dir+"/resources/joint_limits.yaml")

# 打开franka_convert.urdf
修改模块为你自己的绝对路径
 <collision>
      <geometry>
        <mesh filename="file:///home/ziye01/rfmove/resources/franka_description/meshes/collision/link0.stl"/>
      </geometry>
</collision>
将上面filename中的/home/ziye01/rfmove/变换成你自己的franka_description的路径，可以使用Ctrl+H替换所有相关的路径就可以了

```
