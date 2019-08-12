# SOEM for ROS

---

# **IMPORTANT INFORMATION**

**SOEM ROS Package Upgrade Announcement**


To any consumers of the SOEM ROS package:

This package will be upgraded to the new Release of the upstream SOEM repo v1.4.0.
The upgrade via [this PR on GitHub](https://github.com/mgruhler/soem/pull/24) will happen on the

**31st of August 2019**

This upgrade will bring not only the new release, but also changes to the `catkin` plumbing.
This allows you to use soem from within your regular ROS workspace. No more jumping through
the hoops required, that are described in the [Usage section](#Usage).

The changes to the build system are backwards compatible.

Changes in upstream SOEM, however, **might not be**!
Please test the above linked PR and provide
feedback until the merge deadline, if you are a heavy user of this library.

The current state of the SOEM ROS package has been tagged with v1.3.0 if you cannot use the new version.

---

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

This package contains the original SOEM C code provided by the Technische Universiteit Eindhoven,
the development of which has been taken over by [rt-labs](https://rt-labs.com/).
As the original source is down, it is not totally clear what the state of this package is with respect
to the upstream repository.
It is, however, approximately the one merged in [OpenEtherCATsociety/SOEM#1](
https://github.com/OpenEtherCATsociety/SOEM/pull/1).

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
  ## The following work around allows SOEM headers to include other SOEM headers.
  ## SOEM headers assume all headers are installed in a flat directory structure
  ## See https://github.com/mgruhler/soem/issues/4 for more information.
  ${soem_INCLUDE_DIRS}/soem
)
```
**Note** that this assumes you have installed `soem` from `apt` or followed the setup layed out in the [Development](#Development) section!

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

If you want to use `soem` in ROS using a ROS distro it has not been released for, or build it from source,
you need to make sure to use it from an [`install space`](http://wiki.ros.org/catkin/workspaces#Install_Space).
In its current state, this repo is doing some copying of header files during the installation step, which is not done
and thus will not work in regular build/devel space layouts.

Thus, if you want to use `soem` from source, it is encouraged that you put this in an underlay install space.

This has been tested using both, `catkin_make` and `catkin build`.
