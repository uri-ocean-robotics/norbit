# ROS Norbit Multibeam sonar driver

This driver is designed interface directly with a norbit sonar compatible with the [Norbit DFD](https://raw.githubusercontent.com/uri-ocean-robotics/norbit/master/norbit/doc/TN-180196-1D-WBMS_DFD_External.pdf?token=GHSAT0AAAAAABZI2NASRGSDYGYNFR7TZINCY2IJH6Q)

![image](https://user-images.githubusercontent.com/23006525/195664892-8db3c42e-3afb-4a89-9e61-84f3f9fd1ff8.png)

## Installation ##

This project can be installed like any other ROS1 source package

### Begin by getting the package the dependencies ###

First clone the package to your ros workspaces source directory
```
git clone https://github.com/uri-ocean-robotics/norbit.git
```

This package uses the hydrographic_msgs package. Clone them to your workspace's src dircetory

```
git clone https://github.com/apl-ocean-engineering/hydrographic_msgs
```

The rest of the dependencies can be acquired through rosdep.   Run the follwing command from your workspaces root directory.
```
rosdep install --from-paths src --ignore-src -r -y
```

### Compile the package ###
from your catkin workspaces root directory run
```
catkin_make
```

