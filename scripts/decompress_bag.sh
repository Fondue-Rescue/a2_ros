#!/bin/bash

# A function to decompress a zstd-compressed ROS bag file.
# Usage: decompress_zstd_bag.sh <input_bag_file> <output_bag_file>
# Example: decompress_zstd_bag.sh my_bag.mcap.zstd my_bag.mcap

zstd -d "$1" -o "$2"
