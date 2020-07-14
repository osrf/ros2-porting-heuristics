# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import io
import os
import re
import sys

import lxml.etree

import pygount.command


def divide_round_up(n, d):
    return (n + (d - 1)) // d


class File:
    __slots__ = ('name', 'lines')

    def __init__(self, name, lines):
        self.name = name
        self.lines = lines


class LinePlugin():
    __slots__ = ()

    def line_cb(self, line):
        raise Exception('Line callback must be overridden by child class')

    def finish(self, f):
        raise Exception('finish must be overridden by child class')


class RosCPPFileLinePlugin(LinePlugin):
    __slots__ = ('rosfile')

    roscpp_using_re = re.compile(b'using namespace ros')
    roscpp_namespace_re = re.compile(b'ros::')

    def __init__(self):
        self.rosfile = False

    def line_cb(self, line):
        if self.rosfile:
            # No need to do further work if we are already a rosfile
            return

        match = re.search(self.roscpp_using_re, line)
        if match is not None:
            self.rosfile = True
            return

        match = re.search(self.roscpp_namespace_re, line)
        if match is not None:
            self.rosfile = True

    def finish(self, f):
        if not self.rosfile:
            return 0

        return divide_round_up(f.lines, 1000)


class RosCPPUsingTFLinePlugin(LinePlugin):
    __slots__ = ('uses_tf', 'uses_tf2_ros')

    tf_using_re = re.compile(b'tf::')
    tf2_using_re = re.compile(b'tf2_ros::')

    def __init__(self):
        self.uses_tf = False
        self.uses_tf2_ros = False

    def line_cb(self, line):
        if self.uses_tf and self.uses_tf2_ros:
            return

        match = re.search(self.tf_using_re, line)
        if match is not None:
            self.uses_tf = True
            return

        match = re.search(self.tf2_using_re, line)
        if match is not None:
            self.uses_tf2_ros = True

    def finish(self, f):
        if not self.uses_tf:
            return 0

        if self.uses_tf2_ros:
            return 0

        # This means that this file uses tf, but not tf2_ros.  That means
        # we need to do a TF -> TF2 port as part of this file port.
        return 2


class RosCPPUsingDynamicReconfigurePlugin(LinePlugin):
    __slots__ = ('uses_dynamic_reconfigure')

    dynamic_reconfigure_re = re.compile(b'dynamic_reconfigure::')

    def __init__(self):
        self.uses_dynamic_reconfigure = False

    def line_cb(self, line):
        if self.uses_dynamic_reconfigure:
            return

        match = re.search(self.dynamic_reconfigure_re, line)
        if match is not None:
            self.uses_dynamic_reconfigure = True

    def finish(self, f):
        if not self.uses_dynamic_reconfigure:
            return 0

        # This means that this file uses dynamic_reconfigure, so we need
        # to do a port from it to parameters as part of this file port.
        return 2


class RosCPPActionServerPlugin(LinePlugin):
    __slots__ = ('uses_action_server')

    action_server_re = re.compile(b'actionlib::.*Server', re.IGNORECASE)

    def __init__(self):
        self.uses_action_server = False

    def line_cb(self, line):
        if self.uses_action_server:
            return

        match = re.search(self.action_server_re, line)
        if match is not None:
            self.uses_action_server = True

    def finish(self, f):
        if not self.uses_action_server:
            return 0

        # This means that the file uses action_server, so we need to do a port
        # of it to ROS 2
        return 2


class RosPyFileLinePlugin(LinePlugin):
    __slots__ = ('rosfile')

    rospy_re = re.compile(b'rospy')

    def __init__(self):
        self.rosfile = False

    def line_cb(self, line):
        if self.rosfile:
            # No need to do further work if we are already a rosfile
            return

        match = re.search(self.rospy_re, line)
        if match is not None:
            self.rosfile = True
            return

    def finish(self, f):
        if not self.rosfile:
            return 0

        # We count python more heavily than C++ because we also have to
        # likely do a Python 2 -> Python 3 conversion
        return divide_round_up(f.lines, 500)


class RosPyUsingTFLinePlugin(LinePlugin):
    __slots__ = ('uses_tf')

    tf_import_re = re.compile(b'import tf$')
    tf_conversions_import_re = re.compile(b'from tf_conversions import')

    def __init__(self):
        self.uses_tf = False

    def line_cb(self, line):
        if self.uses_tf:
            return

        match = re.search(self.tf_import_re, line)
        if match is not None:
            self.uses_tf = True
            return

        match = re.search(self.tf_conversions_import_re, line)
        if match is not None:
            self.uses_tf = True

    def finish(self, f):
        if not self.uses_tf:
            return 0

        # This means that this file uses tf.  That means we need to do
        # a TF -> TF2 port as part of this file port.
        return 2


class RosPyActionServerLinePlugin(LinePlugin):
    __slots__ = ('uses_action_server')

    action_server_re = re.compile(b'actionlib.*Server', re.IGNORECASE)

    def __init__(self):
        self.uses_action_server = False

    def line_cb(self, line):
        if self.uses_action_server:
            return

        match = re.search(self.action_server_re, line)
        if match is not None:
            self.uses_action_server = True

    def finish(self, f):
        if not self.uses_action_server:
            return 0

        # This means that the file uses action_server, so we need to do a port
        # of it to ROS 2
        return 2


class RosLaunchTestLinePlugin(LinePlugin):
    __slots__ = ('uses_rostest')

    launch_test_re = re.compile(b'<test', re.IGNORECASE)

    def __init__(self):
        self.uses_rostest = False

    def line_cb(self, line):
        if self.uses_rostest:
            return

        match = re.search(self.launch_test_re, line)
        if match is not None:
            self.uses_rostest = True

    def finish(self, f):
        if not self.uses_rostest:
            return 0

        # This means that the file uses rostest, so we need to do a port
        # of it to ROS 2
        return 1


def main():
    parser = argparse.ArgumentParser(description='Utility to walk a source tree and estimate how much effort to port to ROS 2')
    parser.add_argument('--exclusive', help='Only examine the named package (may be passed more than once)', action='append', default=[])
    parser.add_argument('--exclude', help='Exclude the named package (may be passed more than once)', action='append', default=[])
    parser.add_argument('source_path', help='The top-level of the source tree in which to find packages', action='store')
    args = parser.parse_args()

    package_paths = []
    for (dirpath, dirnames, filenames) in os.walk(args.source_path):
        if 'package.xml' not in filenames:
            continue

        package_paths.append(dirpath)

    packages = []
    for package_path in package_paths:
        name = None
        catkin_ignore = False
        if os.path.exists(os.path.join(package_path, 'CATKIN_IGNORE')):
            catkin_ignore = True

        build_depends = 0
        exec_depends = 0
        depends = set()
        tree = lxml.etree.parse(os.path.join(package_path, 'package.xml'))
        for child in tree.getroot().getchildren():
            if child.tag == 'name':
                name = child.text
            elif child.tag == 'build_depend':
                build_depends += 1
                depends.add(child.text)
            elif child.tag == 'exec_depend':
                exec_depends += 1
                depends.add(child.text)
            elif child.tag == 'depend':
                build_depends += 1
                exec_depends += 1
                depends.add(child.text)

        if name is None:
            print("Could not find package name in directory '%s', skipping" % (package_path))
            continue

        if name in args.exclude:
            continue

        if args.exclusive and not name in args.exclusive:
            continue

        sys.stdout = myout = io.StringIO()
        try:
            pygount.command.pygount_command(['-f', 'cloc-xml', package_path])
        finally:
            sys.stdout = sys.__stdout__
        pygount_xml = myout.getvalue()
        pygount_root = lxml.etree.fromstring(pygount_xml)
        files = pygount_root.xpath('/results/files')
        cpp_files = []
        cpp_lines = 0
        py_files = []
        py_lines = 0
        launch_files = []
        msg_files = 0
        if files:
            for child in files[0].getchildren():
                if child.tag != 'file':
                    continue

                language = child.get('language')
                if language in ['C++', 'C']:
                    cpp_files.append(File(child.get('name'), int(child.get('code'))))
                    cpp_lines += int(child.get('code'))
                elif language == '__unknown__':
                    if child.get('name')[-4:] in ['.msg', '.srv']:
                        msg_files += 1
                    elif child.get('name').endswith('.launch'):
                        launch_files.append(File(child.get('name'), int(child.get('code'))))
                elif language == 'Python':
                    py_files.append(File(child.get('name'), int(child.get('code'))))
                    py_lines += int(child.get('code'))

        score = 0

        for cpp_file in cpp_files:
            # We reinitialize the plugins for every file since they may be stateful
            plugins = [RosCPPFileLinePlugin(), RosCPPUsingTFLinePlugin(), RosCPPUsingDynamicReconfigurePlugin(), RosCPPActionServerPlugin()]
            with open(cpp_file.name, 'rb') as infp:
                for line in infp:
                    for plugin in plugins:
                        plugin.line_cb(line)

            for plugin in plugins:
                score += plugin.finish(cpp_file)

        for py_file in py_files:
            # We reinitialize the plugins for every file since they may be stateful
            plugins = [RosPyFileLinePlugin(), RosPyUsingTFLinePlugin(), RosPyActionServerLinePlugin()]
            with open(py_file.name, 'rb') as infp:
                for line in infp:
                    for plugin in plugins:
                        plugin.line_cb(line)

            for plugin in plugins:
                score += plugin.finish(py_file)

        if cpp_files and py_files:
            # For packages that have both C++ and Python, porting to ROS 2
            # becomes a little more tricky.
            score += 1

        for launch_file in launch_files:
            # We reinitialize the plugins for every file since they may be stateful
            plugins = [RosLaunchTestLinePlugin()]
            with open(launch_file.name, 'rb') as infp:
                for line in infp:
                    for plugin in plugins:
                        plugin.line_cb(line)

            for plugin in plugins:
                score += plugin.finish(launch_file)

        score += divide_round_up(len(depends), 2)
        if catkin_ignore:
            size = 'S'
        else:
            if score < 10:
                size = 'S'
            elif score < 30:
                size = 'M'
            else:
                size = 'L'

        packages.append("%s,%d,%d,%d,%d,%d,%d,%d,%s" % (name, build_depends, exec_depends, len(cpp_files), cpp_lines, len(py_files), py_lines, msg_files, size))

    print('Name,# Build Depends,# Exec Depends,# C/C++ files,# C/C++ source lines,# Python files,# Python source lines,# Message/Service files,Porting effort')
    [print(s) for s in sorted(packages)]

    return 0


if __name__ == '__main__':
    sys.exit(main())
