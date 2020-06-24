# SOEM for ROS

**Table of Contents**

- [A Note On The Version Number](#A-Note-On-The-Version-Number)
- [Package Description](#Package-Description)
- [Installation](#Installation)
- [Usage](#Usage)
- [Development](#Development)

## A Note On The Version Number
This package tracks the upstream Open Ethercat Master Repo and thus should
directly reference the respective version number of the upstream.
As this does not allow for intermediate releases that only change ROS specific
parts, like the CMake plumbing, it has been decided to deviate from the upstream
version by adding an arbitrary number (`100`) to the patch part of the version,
and then multiplying by ten (i.e. `(patch of upstream + 100) * 10`).
This allows for intermediate releases in between integrations of upstream releases.

Thus, the version numbers relate to each other as follows:

```
1.4.1000 -> upstream 1.4.0
1.4.1010 -> upstream 1.4.1
1.4.1011 -> upstream 1.4.1 + ROS specific changes 1
```

The idea for this approach was taken from the [cartographer_ros package](https://github.com/ros2/cartographer_ros).

## Package Description

SOEM is an open source EtherCAT master library written in C.
Its primary target is Linux but can be adapted to other OS and embedded systems.

SOEM has originally been hosted at http://developer.berlios.de/projects/soem/
but has been moved to [GitHub and the OpenEtherCATsociety organisation](
https://github.com/OpenEtherCATsociety/SOEM).

This package contains the upstream SOEM repository as a git subtree and wraps it to be easily used within ROS.

**Disclaimer**:
This package is not a development package for SOEM, but rather a wrapper to provide SOEM to ROS.
In the end, this just provides the CMake quirks that allows releasing SOEM as a ROS package.

All bug reports regarding the original SOEM source code should go to the bugtracker at
https://github.com/OpenEtherCATsociety/SOEM/issues.

All ROS related issues should target the [bug tracker on GitHub](https://github.com/mgruhler/soem/issues)
(but might be redirected ;-)).

Obviously, any support, being it bug reports or pull requests (obviously preferred) are highly welcome!

## Installation

If `soem` has been released for your respective ROS distribution, you can simply install it using

```bash
sudo apt install ros-<DISTRO>-soem
```

Currently, `soem` has been released for ROS `indigo`, `kinetic` and `melodic`.
If you want to use `soem` from source, please check out the section about [Development](#Development).

## Usage

To use `soem` in your ROS package add the following to your `package.xml`and `CMakeLists.txt`, respectively.

In your `package.xml` add:

```xml
  <build_depend>soem</build_depend>
  <exec_depend>soem</exec_depend>
```

and in your `CMakeLists.txt`, add it to `find_package` and adapt the `include_directories` as shown:

```CMake
find_package(catkin REQUIRED COMPONENTS
  ...
  soem
  ...
)

include_directories(
  ...
  ${catkin_INCLUDE_DIRS}
)
```

### Running without sudo/root
SOEM requires access to certain network capabilities as it is using raw sockets, and as such any executable linking
against SOEM needs to be run with certain privileges.
Typically, you run any SOEM executables with `sudo` or as `root`.
Tis is impractical for any ROS system, and as such there exists a tool called
[`ethercat_grant`](https://github.com/shadow-robot/ethercat_grant) that helps with that.

Install with
```bash
sudo apt install ros-<DISTRO>-ethercat-ethercat_grant
```
and add the following to your your `node` tag in your launchfile
```xml
launch-prefix="ethercat_grant
```

## Development

With the integration of the upstream SOEM repo as a git subtree, and a major overhaul of the build system,
it is now possible to use the soem ROS package easily from your regular ROS workspace.

Simply clone this repository into your workspace
```bash
git clone git@github.com:mgruhler/soem.git
```

Note that if you want to update or patch the subtree which includes the SOEM upstream repository, you need to be sure
to do this properly.
When creating this, I followed the instructions in
[this Atlassian blog post](https://www.atlassian.com/blog/git/alternatives-to-git-submodule-git-subtree).
This covers all the things you need.

This package has been tested using both, `catkin_make` and `catkin build`.
