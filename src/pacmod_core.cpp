/*
* Unpublished Copyright (c) 2009-2017 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/
  
#include <pacmod_core.h>
#include <bitset>
#include <climits>

using namespace AS::Drivers::PACMod;

const int64_t AS::Drivers::PACMod::AccelCmdMsg::CAN_ID = 0x67;
const int64_t AS::Drivers::PACMod::AccelRptMsg::CAN_ID = 0x68;
const int64_t AS::Drivers::PACMod::GlobalCmdMsg::CAN_ID = 0x69;
const int64_t AS::Drivers::PACMod::GlobalRptMsg::CAN_ID = 0x6A;
const int64_t AS::Drivers::PACMod::BrakeCmdMsg::CAN_ID = 0x6B;                // Delete?
const int64_t AS::Drivers::PACMod::BrakeRptMsg::CAN_ID = 0x6C;
const int64_t AS::Drivers::PACMod::FrontSteerCmdMsg::CAN_ID = 0x6D;
const int64_t AS::Drivers::PACMod::RearSteerCmdMsg::CAN_ID = 0x6E;
const int64_t AS::Drivers::PACMod::VehicleSpeedRptMsg::CAN_ID = 0x6F;
const int64_t AS::Drivers::PACMod::VehicleSpeedCmdMsg::CAN_ID = 0x9A; 	      //Edited to command EPEC
const int64_t AS::Drivers::PACMod::BrakeMotorRpt1Msg::CAN_ID = 0x70;
const int64_t AS::Drivers::PACMod::BrakeMotorRpt2Msg::CAN_ID = 0x71;
const int64_t AS::Drivers::PACMod::BrakeMotorRpt3Msg::CAN_ID = 0x72;
const int64_t AS::Drivers::PACMod::SteerMotorRpt1Msg::CAN_ID = 0x73;
const int64_t AS::Drivers::PACMod::SteerMotorRpt2Msg::CAN_ID = 0x74;
const int64_t AS::Drivers::PACMod::SteerMotorRpt3Msg::CAN_ID = 0x75;
#if 0
const int64_t AS::Drivers::PACMod::HeadlightCmdMsg::CAN_ID = 0x76;
const int64_t AS::Drivers::PACMod::HeadlightRptMsg::CAN_ID = 0x77;
const int64_t AS::Drivers::PACMod::HornCmdMsg::CAN_ID = 0x78;
const int64_t AS::Drivers::PACMod::HornRptMsg::CAN_ID = 0x79;
#endif
const int64_t AS::Drivers::PACMod::WheelSpeedRptMsg::CAN_ID = 0x7A;
const int64_t AS::Drivers::PACMod::SteeringPIDRpt1Msg::CAN_ID = 0x7B;
const int64_t AS::Drivers::PACMod::SteeringPIDRpt2Msg::CAN_ID = 0x7C;
const int64_t AS::Drivers::PACMod::SteeringPIDRpt3Msg::CAN_ID = 0x7D;
const int64_t AS::Drivers::PACMod::SteeringPIDRpt4Msg::CAN_ID = 0x84;
const int64_t AS::Drivers::PACMod::SteerRpt2Msg::CAN_ID = 0x7E;               // IN USE
const int64_t AS::Drivers::PACMod::SteerRpt3Msg::CAN_ID = 0x7F;               // IN USE
/************************* Message by Joon ****************************/
const int64_t AS::Drivers::PACMod::PIDTuningCmdMsg::CAN_ID = 0x42;
const int64_t AS::Drivers::PACMod::PIDTuningCmdRptMsg::CAN_ID = 0x43;
const int64_t AS::Drivers::PACMod::ControlModeMsg::CAN_ID = 0x02;
const int64_t AS::Drivers::PACMod::HeartbeatVCUMsg::CAN_ID = 0x05;
#if 0 
const int64_t AS::Drivers::PACMod::ParkingBrakeStatusRptMsg::CAN_ID = 0x80; 
#endif
const int64_t AS::Drivers::PACMod::YawRateRptMsg::CAN_ID = 0x81;
const int64_t AS::Drivers::PACMod::LatLonHeadingRptMsg::CAN_ID = 0x82;
const int64_t AS::Drivers::PACMod::DateTimeRptMsg::CAN_ID = 0x83;
#if 0 
const int64_t AS::Drivers::PACMod::VinRptMsg::CAN_ID = 0xFF;                  // Chenge to AEV vin
#endif

/*******************************************************************************/
std::shared_ptr<PacmodTxMsg> PacmodTxMsg::make_message(const int64_t& can_id)
{
  switch (can_id)
  {
  case AccelRptMsg::CAN_ID:
    return std::shared_ptr<PacmodTxMsg>(new AccelRptMsg);
    break;
  case GlobalRptMsg::CAN_ID:
    return std::shared_ptr<PacmodTxMsg>(new GlobalRptMsg);
    break;
  case BrakeRptMsg::CAN_ID:
    return std::shared_ptr<PacmodTxMsg>(new BrakeRptMsg);
    break;
  case VehicleSpeedRptMsg::CAN_ID:
    return std::shared_ptr<PacmodTxMsg>(new VehicleSpeedRptMsg);
    break;
  case BrakeMotorRpt1Msg::CAN_ID:
    return std::shared_ptr<PacmodTxMsg>(new BrakeMotorRpt1Msg);
    break;
  case BrakeMotorRpt2Msg::CAN_ID:
    return std::shared_ptr<PacmodTxMsg>(new BrakeMotorRpt2Msg);
    break;
  case BrakeMotorRpt3Msg::CAN_ID:
    return std::shared_ptr<PacmodTxMsg>(new BrakeMotorRpt3Msg);
    break;
  case SteerMotorRpt1Msg::CAN_ID:
    return std::shared_ptr<PacmodTxMsg>(new SteerMotorRpt1Msg);
    break;
  case SteerMotorRpt2Msg::CAN_ID:
    return std::shared_ptr<PacmodTxMsg>(new SteerMotorRpt2Msg);
    break;
  case SteerMotorRpt3Msg::CAN_ID:
    return std::shared_ptr<PacmodTxMsg>(new SteerMotorRpt3Msg);
    break;
#if 0
  case HeadlightRptMsg::CAN_ID:
    return std::shared_ptr<PacmodTxMsg>(new HeadlightRptMsg);
    break;
  case HornRptMsg::CAN_ID:
    return std::shared_ptr<PacmodTxMsg>(new HornRptMsg);
    break;
#endif
  case WheelSpeedRptMsg::CAN_ID:
    return std::shared_ptr<PacmodTxMsg>(new WheelSpeedRptMsg);
    break;
  case SteeringPIDRpt1Msg::CAN_ID:
    return std::shared_ptr<PacmodTxMsg>(new SteeringPIDRpt1Msg);
    break;
  case SteeringPIDRpt2Msg::CAN_ID:
    return std::shared_ptr<PacmodTxMsg>(new SteeringPIDRpt2Msg);
    break;
  case SteeringPIDRpt3Msg::CAN_ID:
    return std::shared_ptr<PacmodTxMsg>(new SteeringPIDRpt3Msg);
    break;
  case SteerRpt2Msg::CAN_ID:
    return std::shared_ptr<PacmodTxMsg>(new SteerRpt2Msg);
    break;
  case SteerRpt3Msg::CAN_ID:
    return std::shared_ptr<PacmodTxMsg>(new SteerRpt3Msg);
    break;
#if 0
  case ParkingBrakeStatusRptMsg::CAN_ID:
    return std::shared_ptr<PacmodTxMsg>(new ParkingBrakeStatusRptMsg);
    break;
#endif
  case YawRateRptMsg::CAN_ID:
    return std::shared_ptr<PacmodTxMsg>(new YawRateRptMsg);
    break;
  case LatLonHeadingRptMsg::CAN_ID:
    return std::shared_ptr<PacmodTxMsg>(new LatLonHeadingRptMsg);
    break;
  case DateTimeRptMsg::CAN_ID:
    return std::shared_ptr<PacmodTxMsg>(new DateTimeRptMsg);
    break;
  case SteeringPIDRpt4Msg::CAN_ID:
    return std::shared_ptr<PacmodTxMsg>(new SteeringPIDRpt4Msg);
    break;
  case PIDTuningCmdRptMsg::CAN_ID:
    return std::shared_ptr<PacmodTxMsg>(new PIDTuningCmdRptMsg);
    break;
#if 0
  case VinRptMsg::CAN_ID:
    return std::shared_ptr<PacmodTxMsg>(new VinRptMsg);
    break;  
#endif
  default:
    return NULL;
  }
}

// ***************************** TX Messages ************************************
void GlobalRptMsg::parse(uint8_t *in)
{
  enabled = in[0] & 0x01;
  override_active = ((in[0] & 0x02) >> 1) != 0;
  user_can_timeout = ((in[0] & 0x20) >> 5) != 0;
  brake_can_timeout = ((in[0] & 0x10) >> 4) != 0;
  front_steering_can_timeout = ((in[0] & 0x08) >> 3) != 0;
  rear_steering_can_timeout = ((in[0] & 0x08) >> 3) != 0;
  vehicle_can_timeout = ((in[0] & 0x04) >> 2) != 0;
  user_can_read_errors = ((in[6] << 8) | in[7]);
}


void SystemRptIntMsg::parse(uint8_t *in)
{
  manual_input = in[0];
  command = in[1];
  output = in[2];
}

void SystemRptFloatMsg::parse(uint8_t *in)
{
  int16_t temp;

  temp = (static_cast<int16_t>(in[0]) << 8) | in[1];
  manual_input = static_cast<double>(temp / 1000.0);

  temp = (static_cast<int16_t>(in[2]) << 8) | in[3];
  command = static_cast<double>(temp / 1000.0);

  temp = (static_cast<int16_t>(in[4]) << 8) | in[5];
  output = static_cast<double>(temp / 1000.0);
}

void VehicleSpeedRptMsg::parse(uint8_t *in)
{
  int16_t temp;

  temp = (static_cast<int16_t>(in[0]) << 8) | in[1];
  vehicle_speed = static_cast<double>(temp / 100.0);

  vehicle_speed_valid = (in[2] == 1);
  vehicle_speed_raw[0] = in[3];
  vehicle_speed_raw[1] = in[4];
}

void YawRateRptMsg::parse(uint8_t *in)
{
  int16_t temp;

  temp = (static_cast<int16_t>(in[0]) << 8) | in[1];
  yaw_rate = static_cast<double>(temp / 100.0);
}

void LatLonHeadingRptMsg::parse(uint8_t *in)
{
  latitude_degrees = static_cast<int8_t>(in[0]);
  latitude_minutes = in[1];
  latitude_seconds = in[2];
  longitude_degrees = static_cast<int8_t>(in[3]);
  longitude_minutes = in[4];
  longitude_seconds = in[5];
  heading = ((static_cast<int16_t>(in[6]) << 8) | in[7]) / 100.0;
}

void DateTimeRptMsg::parse(uint8_t *in)
{
  year = in[0];
  month = in[1];
  day = in[2];
  hour = in[3];
  minute = in[4];
  second = in[5];
}

void WheelSpeedRptMsg::parse(uint8_t *in)
{
  int16_t temp;

  temp = (static_cast<int16_t>(in[0]) << 8) | in[1];
  front_left_wheel_speed = static_cast<double>(temp / 100.0);

  temp = (static_cast<int16_t>(in[2]) << 8) | in[3];
  front_right_wheel_speed = static_cast<double>(temp / 100.0);

  temp = (static_cast<int16_t>(in[4]) << 8) | in[5];
  rear_left_wheel_speed = static_cast<double>(temp / 100.0);

  temp = (static_cast<int16_t>(in[6]) << 8) | in[7];
  rear_right_wheel_speed = static_cast<double>(temp / 100.0);
}

void MotorRpt1Msg::parse(uint8_t *in)
{
  int32_t temp;

  temp = (static_cast<int32_t>(in[0]) << 24) |
         (static_cast<int32_t>(in[1]) << 16) |
         (static_cast<int32_t>(in[2]) << 8) | in[3];
  current = static_cast<double>(temp / 1000.0);

  temp = (static_cast<int32_t>(in[4]) << 24) |
         (static_cast<int32_t>(in[5]) << 16) |
         (static_cast<int32_t>(in[6]) << 8) | in[7];
  position = static_cast<double>(temp / 1000.0);
}

void MotorRpt2Msg::parse(uint8_t *in)
{
  int16_t temp16;
  int32_t temp32;

  temp16 = (static_cast<int16_t>(in[0]) << 8) | in[1];
  encoder_temp = static_cast<double>(temp16);

  temp16 = (static_cast<int16_t>(in[2]) << 8) | in[3];
  motor_temp = static_cast<double>(temp16);

  temp32 = (static_cast<int32_t>(in[7]) << 24) |
           (static_cast<int32_t>(in[6]) << 16) |
           (static_cast<int32_t>(in[5]) << 8) | in[4];
  velocity = static_cast<double>(temp32 / 1000.0);
}

void MotorRpt3Msg::parse(uint8_t *in)
{
  int32_t temp;

  temp = (static_cast<int32_t>(in[0]) << 24) |
         (static_cast<int32_t>(in[1]) << 16) |
         (static_cast<int32_t>(in[2]) << 8) | in[3];
  torque_output = static_cast<double>(temp / 1000.0);

  temp = (static_cast<int32_t>(in[4]) << 24) |
         (static_cast<int32_t>(in[5]) << 16) |
         (static_cast<int32_t>(in[6]) << 8) | in[7];
  torque_input = static_cast<double>(temp / 1000.0);
}

void SteeringPIDRpt1Msg::parse(uint8_t *in)
{
  int16_t temp;

  temp = (static_cast<int16_t>(in[0]) << 8) | in[1];
  dt = static_cast<double>(temp / 1000.0);

  temp = (static_cast<int16_t>(in[2]) << 8) | in[3];
  Kp = static_cast<double>(temp / 1000.0);

  temp = (static_cast<int16_t>(in[4]) << 8) | in[5];
  Ki = static_cast<double>(temp / 1000.0);

  temp = (static_cast<int16_t>(in[6]) << 8) | in[7];
  Kd = static_cast<double>(temp / 1000.0);
}

void SteeringPIDRpt2Msg::parse(uint8_t *in)
{
  int16_t temp;

  temp = (static_cast<int16_t>(in[0]) << 8) | in[1];
  P_term = static_cast<double>(temp / 1000.0);

  temp = (static_cast<int16_t>(in[2]) << 8) | in[3];
  I_term = static_cast<double>(temp / 1000.0);

  temp = (static_cast<int16_t>(in[4]) << 8) | in[5];
  D_term = static_cast<double>(temp / 1000.0);

  temp = (static_cast<int16_t>(in[6]) << 8) | in[7];
  all_terms = static_cast<double>(temp / 1000.0);
}

void SteeringPIDRpt3Msg::parse(uint8_t *in)
{
  int16_t temp;

  temp = (static_cast<int16_t>(in[0]) << 8) | in[1];
  new_torque = static_cast<double>(temp / 1000.0);

  temp = (static_cast<int16_t>(in[2]) << 8) | in[3];
  str_angle_desired = static_cast<double>(temp / 1000.0);

  temp = (static_cast<int16_t>(in[4]) << 8) | in[5];
  str_angle_actual = static_cast<double>(temp / 1000.0);

  temp = (static_cast<int16_t>(in[6]) << 8) | in[7];
  error = static_cast<double>(temp / 1000.0);
}

void SteeringPIDRpt4Msg::parse(uint8_t *in)
{
  int16_t temp;

  temp = (static_cast<int16_t>(in[0]) << 8) | in[1];
  angular_velocity = static_cast<double>(temp / 1000.0);

  temp = (static_cast<int16_t>(in[2]) << 8) | in[3];
  angular_acceleration = static_cast<double>(temp / 1000.0);
}

void PIDTuningCmdRptMsg::parse(uint8_t *in)
{
  uint8_t *temp;
  temp = (uint8_t *)&value;  

  selection = static_cast<uint8_t>(in[0]);
  term = static_cast<uint8_t>(in[1]);
  temp[0] = in[5];
  temp[1] = in[4];
  temp[2] = in[3];
  temp[3] = in[2];
}

#if 0
void ParkingBrakeStatusRptMsg::parse(uint8_t *in)
{
  parking_brake_engaged = (in[0] == 0x01);
}

#endif

// RX Messages
void GlobalCmdMsg::encode(bool enable, bool clear_override, bool ignore_overide)
{
  data.assign(8, 0);

  if (enable)
    data[0] |= 0x01;
  if (clear_override)
    data[0] |= 0x02;
  if (ignore_overide)
    data[0] |= 0x04;
}

void ControlModeMsg::encode(bool autopilot, bool can_active)
{
  data.assign(8,0);
  data[0] = autopilot;
}

void VehicleSpeedCmdMsg::encode(double vehicle_speed, bool vehicle_speed_valid)
{
  int32_t out;

  data.assign(8,0);


  /* Below is the code as implemented by Mitch */
  // Untested code, unsure if encoding for EPEC is w mask or w/o mask
  out = (uint32_t)(1000.0*vehicle_speed);
  data[0] = (out & 0xFF000000) >> 24;
  data[1] = (out & 0x00FF0000) >> 16;
  data[2] = (out & 0x0000FF00) >> 8;
  data[3] = out & 0x000000FF; 
  data[4] = vehicle_speed_valid;
}

void PIDTuningCmdMsg::encode(uint8_t selection, uint8_t term, uint8_t oper, float value)
{
  data.assign(8,0);

  union{
    float input;
    int output;
  } temp;

  temp.input = value;
  std::bitset<sizeof(float) * CHAR_BIT> bits(temp.output);

  unsigned long int out = bits.to_ullong(); 

  data[0] = selection;
  data[1] = term;
  data[2] = oper;
  data[3] = (out & 0xFF000000) >> 24;
  data[4] = (out & 0x00FF0000) >> 16;
  data[5] = (out & 0x0000FF00) >> 8;
  data[6] = out & 0x000000FF;
}

#if 0
void HeadlightCmdMsg::encode(uint8_t headlight_cmd)
{
  data.assign(8, 0);
  data[0] = headlight_cmd;
}

void HornCmdMsg::encode(uint8_t horn_cmd)
{
  data.assign(8, 0);
  data[0] = horn_cmd;
}
#endif

void HeartbeatVCUMsg::encode(uint16_t expected_msg)
{
  data.assign(8,0);
  data[0] = 0x000000FF & expected_msg; 
}

void AccelCmdMsg::encode(double accel_cmd)
{
  data.assign(8, 0);
  uint16_t cmdInt = static_cast<uint16_t>(accel_cmd * 1000.0);
  data[0] = (cmdInt & 0xFF00) >> 8;
  data[1] = cmdInt & 0x00FF;
}

void FrontSteerCmdMsg::encode(double steer_pos, double steer_spd)
{
  data.assign(8, 0);
  int32_t raw_pos = static_cast<int32_t>(1000.0 * steer_pos);
  uint32_t raw_spd = (uint32_t)(1000.0 * steer_spd);

  data[0] = (raw_pos & 0xFF000000) >> 24;
  data[1] = (raw_pos & 0x00FF0000) >> 16;
  data[2] = (raw_pos & 0x0000FF00) >> 8;
  data[3] = raw_pos & 0x000000FF;
  data[4] = (raw_spd & 0xFF000000) >> 24;
  data[5] = (raw_spd & 0x00FF0000) >> 16;
  data[6] = (raw_spd & 0x0000FF00) >> 8;
  data[7] = raw_spd & 0x000000FF;
}

void RearSteerCmdMsg::encode(double steer_pos, double steer_spd)
{
  data.assign(8, 0);
  int32_t raw_pos = static_cast<int32_t>(1000.0 * steer_pos);
  uint32_t raw_spd = (uint32_t)(1000.0 * steer_spd);

  data[0] = (raw_pos & 0xFF000000) >> 24;
  data[1] = (raw_pos & 0x00FF0000) >> 16;
  data[2] = (raw_pos & 0x0000FF00) >> 8;
  data[3] = raw_pos & 0x000000FF;
  data[4] = (raw_spd & 0xFF000000) >> 24;
  data[5] = (raw_spd & 0x00FF0000) >> 16;
  data[6] = (raw_spd & 0x0000FF00) >> 8;
  data[7] = raw_spd & 0x000000FF;
}

void BrakeCmdMsg::encode(double brake_pct)
{
  data.assign(8, 0);
  uint16_t raw_pct = static_cast<uint16_t>(1000.0 * brake_pct);

  data[0] = (raw_pct & 0xFF00) >> 8;
  data[1] = (raw_pct & 0x00FF);
}

