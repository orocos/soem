# SOEM for ROS

**Table of Contents**

- [Package Description](#Package-Description)
- [Installation](#Installation)
- [Usage](#Usage)
- [Development](#Development)

## Package Description

SOEM is an open source EtherCAT master library written in C.
Its primary target is Linux but can be adapted to other OS and embedded systems.

SOEM has originally been hosted at http://developer.berlios.de/projects/soem/
but has been moved to [GitHub and the OpenEtherCATsociety organisation](
https://github.com/OpenEtherCATsociety/SOEM).

This package contains the upstream SOEM repository as a git submodule and wraps it to be easily used within ROS.

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

## Development

With the integration of the upstream SOEM repo as a git submodule, and a major overhaul of the build system,
it is now possible to use the soem ROS package easily from your regular ROS workspace.

You only need to make sure to clone this repository with the submodules, i.e. either call
```bash
git clone --recursive git@github.com:mgruhler/soem.git
```
or do a regular clone with a manual initialization of the git submodule
```
git clone git@github.com:mgruhler/soem.git
git submodule init
git submodule update
```

This package has been tested using both, `catkin_make` and `catkin build`.
