## AEV PACMod fork

This fork is to allow for the development of aev features using the AutonomouStuff PACMod code base. Development is undertaken on branch, to allow for a pull request at some point.

Signals available prior to fork, commit [165b742bd655413dba06e335aaf0e2fde921d81e](https://github.com/astuff/pacmod/commit/165b742bd655413dba06e335aaf0e2fde921d81e)

 ```cpp 
 // TX, complete set
 BrakeCmdMsg                static_cast<uint16_t>(1000.0 * brake_pct)
 SteerCmdMsg                static_cast<int32_t>(1000.0 * steer_pos), (uint32_t)(1000.0 * steer_spd)
 AccelCmdMsg                static_cast<uint16_t>(accel_cmd * 1000.0)
 ShiftCmdMsg                uint8_t shift_cmd
 WiperCmdMsg                uint8_t wiper_cmd
 HornCmdMsg                 uint8_t horn_cmd
 HeadlightCmdMsg            uint8_t headlight_cmd
 TurnSignalCmdMsg           uint8_t turn_signal_cmd
 GlobalCmdMsg               bool enable, bool clear_override, bool ignore_overide

 // RX, complete set
 ParkingBrakeStatusRptMsg   bool parking_brake_engaged
 SteeringPIDRpt4Msg         double angular_velocity, double angular_acceleration
 SteeringPIDRpt3Msg         double new_torque, double str_angle_desired, double str_angle_actual, double error
 SteeringPIDRpt2Msg         double P_term, double I_term, double D_term, double all_terms
 SteeringPIDRpt1Msg         double dt, double Kp, double Ki, double Kd
 MotorRpt3Msg               double torque_input, double torque_output
 MotorRpt2Msg               double encoder_temp, double motor_temp, double velocity
 MotorRpt1Msg               double current, double position
 WheelSpeedRptMsg           double front_left_wheel_speed, double front_right_wheel_speed
                            double rear_left_wheel_speed, double rear_right_wheel_speed
 DateTimeRptMsg             uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second
 LatLonHeadingRptMsg        uint8_t latitude_degrees, uint8_t latitude_minutes, uint8_t latitude_seconds, 
                            uint8_t latitude_degrees, uint8_t latitude_minutes, uint8_t latitude_seconds,
                            uint16_t heading
 YawRateRptMsg              double yaw_rate
 VehicleSpeedRptMsg         double vehicle_speed, bool vehicle_speed_valid, uint16_t vehicle_speed_raw
 SystemRptFloatMsg          double manual_input, double command, double output
 SystemRptIntMsg            uint8_t manual_input, uint8_t command, uint8_t output
 GlobalRptMsg               bool enabled, bool override_active, bool user_can_timeout, bool brake_can_timeout,
                            bool steering_can_timeout, bool vehicle_can_timeout, bool user_can_read_errors
 
 // Default RX                          TX
            GlobalRptMsg                GlobalCmdMsg
            VinRptMsg                   
            TurnSignalRptMsg            TurnSignalCmdMsg
            ShiftRptMsg                 ShiftCmdMsg
            AccelRptMsg                 AccelCmdMsg
            SteerRptMsg                 SteerCmdMsg
            BrakeRptMsg                 BrakeCmdMsg
            VehicleSpeedRptMsg          

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
