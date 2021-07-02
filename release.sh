#!/bin/bash

if [ $# -lt 1 ]
then
  ver=""
else
  ver="$1."
fi

extern_lib_path="extern/lib"
install_path="install"

echo "Set extern_lib_path to ${extern_lib_path}"
echo "Set install_path to ${install_path}"

if [ ! -d "${install_path}/moveit_no_ros/lib" ]
then
  echo "Directory ${install_path}/moveit_no_ros/lib not exists."
  echo "Make directory ${install_path}/moveit_no_ros/lib."
  mkdir -p ${install_path}/moveit_no_ros/lib
fi

if [ ! -d "${install_path}/moveit_no_ros/resources" ]
then
  echo "Directory ${install_path}/moveit_no_ros/resources not exists."
  echo "Make directory ${install_path}/moveit_no_ros/resources."
  mkdir -p ${install_path}/moveit_no_ros/resources
fi

cd extern/lib
bash rmlink.sh
cd ../..

cp -r ${extern_lib_path}/* ${install_path}/moveit_no_ros/lib/
cp ${install_path}/lib/* ${install_path}/moveit_no_ros/lib/
cp -r resources/pr2_description ${install_path}/moveit_no_ros/resources
cp -r resources/pr2_config ${install_path}/moveit_no_ros/resources
cp resources/pr2.urdf ${install_path}/moveit_no_ros/resources

cd ${install_path}
rm -f moveit_no_ros*.tar.gz
tar -czf moveit_no_ros.${ver}tar.gz moveit_no_ros
cd ..

cd extern/lib
bash mklink.sh
cd ../..
