#!/bin/bash

#---Automatically Generated from template 'bash' wrote by @aliben---
# @Copyright (C) 2019 All rights reserved.
# @file: run_carto_customized.bash
# @author: aliben.develop@gmail.com
# @created_date: 2019-06-21 16:54:01
# @last_modified_date: 2019-07-19 08:55:49
# @brief: TODO
# @details: TODO
#---***********************************************---


#---Variables
CREATED_TIME=`date '+%Y-%m-%d %H:%M:%S'`
CREATED_YEAR=`date '+%Y'`


if [ "$#" -lt "1" ]; then
  echo "Usage: $0 ABSOLUTE_PATH_ROS_BAG [true|false]"
  echo "ABSOLUTE_PATH_ROS_BAG is the absolute path of a ros bag"
  echo "LANDMARKS_LOCATION_FILENAME is the absolute path of landmarks original coordinates"
  echo "If rviz is not neccessary for building stage, the 3nd params might be set with true. The DEFAULT is true to disable rviz"
  exit 1
fi
mkdir -p $HOME/USERNAME/map
mkdir -p $HOME/USERNAME/config_landmarks
if [ "$3" == "debug" ]; then
  LAUNCH_PREFIX="xterm -e gdb -ex run --args"
fi

BAG=$1
BAG1=`echo $BAG|cut -d "," -f1`
MNAME="${BAG1}_`date '+%Y%m%d_%H%M%S'`"
SAVE_FILENAME=$HOME/USERNAME/map/`basename $MNAME`_raw_landmarks_data.txt
EN=$2
if [ "$2" == "" ]; then
  EN="true"
  echo "no default"
fi
#---Shell Command

EXT=".pbstream"
trap "exit 1" INT

roslaunch cartographer_ros offline_backpack_2d_vn.launch bag_filenames:=$1 launch_prefix:="$LAUNCH_PREFIX" save_filename:=$SAVE_FILENAME GL:=true no_rviz:=$EN
sort -n $SAVE_FILENAME -o $SAVE_FILENAME
rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename ${BAG1}${EXT} -map_filestem $HOME/USERNAME/map/`basename $MNAME` -resolution 0.025&&

echo ""
echo ""
echo "Generation of Map DONE!!!!!!!!!"
echo "map path: $MNAME"
echo "$HOME/USERNAME/map/`basename $MNAME`" > $HOME/USERNAME/map_filename
echo "$BAG1" > $HOME/USERNAME/bag_filename


TMP_NAME=`cat $HOME/USERNAME/map_filename`
POST_SAVE_NAME="$HOME/USERNAME/map/${TMP_NAME##*/}_poster_landmarks.txt"
rosrun cartographer_ros cartographer_poster_landmark $SAVE_FILENAME $HOME/USERNAME/map/`basename $MNAME`_poster_landmarks.txt
echo "$POST_SAVE_NAME" > $HOME/USERNAME/poster_filename
