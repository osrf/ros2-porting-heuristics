# ROS 2 Porting Heuristics

This repository contains a script to estimate the amount of work needed to port
a set of packages from ROS 1 to ROS 2.  It will crawl a source directory looking
for ROS packages.  For each package, it will apply a set of heuristics to
determine how much of ROS 1 it uses and how much work those parts are to port
to ROS 2.  Based on this information, it will output a CSV line with the name of
the package, various statistics about the package, and a final "score" of
Small, Medium, or Large for the porting effort.

## Dependencies

This script relies on lxml (which is available on pretty much any Linux
distribution under the name `python3-lxml` or similar), and
[`pygount`](https://pypi.org/project/pygount/), a library for analyzing source
code.

## Usage

python3 ros2-porting-heuristics.py /path/to/src
