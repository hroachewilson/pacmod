## AEV PACMod fork

This fork is to allow for the development of aev features using the AutonomouStuff PACMod code base. Development is undertaken on branch, to allow for a pull request at some point.

Signals available via pacmod

 ```cpp 
 
 BrakeCmdMsg    static_cast<uint16_t>(1000.0 * brake_pct)
 SteerCmdMsg    static_cast<int32_t>(1000.0 * steer_pos), (uint32_t)(1000.0 * steer_spd)

 
 
 ```

## PACMod (Platform Actuation and Control MODule) Vehicle Interface #

[![CircleCI](https://circleci.com/gh/astuff/pacmod/tree/master.svg?style=svg)](https://circleci.com/gh/astuff/pacmod/tree/master)

This ROS node is designed to allow the user to control a vehicle (see SUPPORTED VEHICLES below) with the PACMod drive-by-wire system. For more information about the topics, parameters, and details on the implementation, see [the ROS wiki](http://wiki.ros.org/pacmod).

For access to the DBC file which defines the CAN interface for the PACMod, see the [pacmod1_2_dbc](https://github.com/astuff/pacmod1_2_dbc) repo.

## Supported Vehicles ##

- Polaris GEM Series (e2/e4/e6) MY 2016+
- Polaris eLXD MY 2016+
- Polaris Ranger X900
- International Prostar+ 122
- Lexus RX-450h MY 2016+
- AEV Development Mule
- More coming soon...
