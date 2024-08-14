# mc_unitree2
Interface between [Unitree robots](https://github.com/unitreerobotics/unitree_ros2/tree/master/robots) and [mc_rtc](https://jrl-umi3218.github.io/mc_rtc). Provides connectivity with [Go2](https://www.unitree.com/products/go2/) robots.

## 1. Required dependencies

 - [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/)
 - [mc_go2_description](https://github.com/isri-aist/mc_go2_description)
 - [mc_go2](https://github.com/isri-aist/mc_go2)
 - [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)

## 2. Install dependencies

### unitree_sdk2
 - Install the include files
```
$ git clone https://github.com/unitreerobotics/unitree_sdk2
$ mkdir build
$ cmake ..
$ make ; make install
```
## 3. Install this project

### Build instructions

```
$ cd src
$ git clone https://github.com/@MC_UNITREE@
$ cd @MC_UNITREE_PATH@
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
```

### The path set to the ROS library

`MCControlUnitree2` will not pick up ROS libraries. If you're using `mc_rtc`'s ROS plugin, create a file with content: `/etc/ld.so.conf.d/ros.conf`
```
/opt/ros/${ROS_DISTRO}/lib
```
Run the following command after this change:
```
$ sudo ldconfig
```

## 4. Usage
To use the interface and connect to a real robot run

```
$ MCControlUnitree --help

 MCControlUnitree options:
   --help                                display help message
   -h [ --host ] arg (=go2/h1)           connection host, robot name or
                                         "simulation"
   -f [ --conf ] arg (=/usr/local/etc/mc_unitree/mc_rtc_go2.yaml)
                                         configuration file

$ MCControlUnitree -h <robot_hostname> -f <mc_rtc_configuration_file.conf>
```

Where <mc_rtc_configuration_file.yaml> is based on (e.g).

 `<INSTALL_PREFIX>/etc/mc_unitree/<robot>.yaml` --> `/usr/local/etc/mc_unitree/mc_rtc_go2.yaml`

If you wish to run the simulation only use as a simulation (replace the `<robot_hostname>` with simulation)

```
$ MCControlUnitree -h simulation
```

Your mc_rtc configuration file (typically ) should contain the following lines: `/usr/local/etc/mc_unitree/mc_rtc_go2.yaml`

```
# Interface specific parameters (mc_unitree)
Unitree:
  go2: # Name of the robot in the controller
    network: eth0 # Name of network interface
```

Run the program:

```
$ MCControlUnitree2
```

You can also provide an additional configuration file (to swap between different network configurations easily for example):

```
$ MCControlUnitree2 -f conf.yaml
```
