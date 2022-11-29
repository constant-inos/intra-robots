#!/bin/bash
my_models_folder='./gazebo_models_using'
gazebo_models_folder='/usr/share/gazebo-11/models/'

for object in $my_models_folder/*;
do
cp -r $object $gazebo_models_folder;
f="$(basename -- $object)"
sudo chmod -R +rwx $gazebo_models_folder/$f
done

my_world_path='./worlds'
gazebo_world_path='/home/karagk/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_gazebo'
cp -r $my_world_path $gazebo_world_path;
sudo chmod -R +rwx $gazebo_world_path/worlds
