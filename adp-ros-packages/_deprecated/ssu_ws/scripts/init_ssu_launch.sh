#!/bin/bash

launch_path=../src/launch
ssu_launch_path=../src/launch/ssu_launch

if [ -d $ssu_launch_path ]; then
    echo "ssu_launch already exists"
else
    ros2 pkg create --build-type ament_cmake --dependencies \
        rviz2 \
        ssu_control_launch \
        ssu_perception_launch \
        ssu_planning_launch \
        ssu_sensing_launch \
        ssu_map_launch \
        ssu_simulator_launch \
        ssu_system_launch \
        ssu_vehicle_launch \
        --description "ssu_launch" --destination-directory $launch_path \
        ssu_launch

    sed -i 's/<depend>/<exec_depend>/g; s/<\/depend>/<\/exec_depend>/g' $ssu_launch_path/package.xml
fi

if [ -d $ssu_launch_path/src ]; then
    rm -rf $ssu_launch_path/src
fi

if [ -d $ssu_launch_path/include ]; then
    rm -rf $ssu_launch_path/include
fi

touch $ssu_launch_path/README.md

mkdir -p $ssu_launch_path/config
mkdir -p $ssu_launch_path/config/control
touch $ssu_launch_path/config/control/default_preset.yaml
mkdir -p $ssu_launch_path/config/map
mkdir -p $ssu_launch_path/config/perception
mkdir -p $ssu_launch_path/config/planning
mkdir -p $ssu_launch_path/config/simulator
mkdir -p $ssu_launch_path/config/system

mkdir -p $ssu_launch_path/launch
touch $ssu_launch_path/launch/ssu.launch.xml
mkdir -p $ssu_launch_path/launch/components
touch $ssu_launch_path/launch/components/ssu_control_component.launch.xml
touch $ssu_launch_path/launch/components/ssu_map_component.launch.xml
touch $ssu_launch_path/launch/components/ssu_perception_component.launch.xml
touch $ssu_launch_path/launch/components/ssu_planning_component.launch.xml
touch $ssu_launch_path/launch/components/ssu_simulator_component.launch.xml
touch $ssu_launch_path/launch/components/ssu_system_component.launch.xml

mkdir -p $ssu_launch_path/rviz
touch $ssu_launch_path/rviz/ssu.rviz
mkdir -p $ssu_launch_path/rviz/image