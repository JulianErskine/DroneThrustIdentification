#!/bin/bash

msgs='actuator_controls_0,actuator_outputs,sensor_combined,battery_status,vehicle_attitude,input_rc'
echo "Enter a .ulg file name: "
read -e file_name

if test -f "$file_name.ulg"; then
    echo "file $file_name.ulg exists, extracting csv..."
    ulog2csv -m $msgs $file_name.ulg    # Run ulg to csv
	mkdir -p $file_name 				# create directory to place new files
	mv $file_name*.csv $file_name
else
    echo "There is no file of that name"
fi


