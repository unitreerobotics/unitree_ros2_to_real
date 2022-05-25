/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef _CONVERT_H_
#define _CONVERT_H_

#include "ros2_unitree_legged_msgs/msg/low_cmd.h"
#include "ros2_unitree_legged_msgs/msg/low_state.h"
#include "ros2_unitree_legged_msgs/msg/high_cmd.h"
#include "ros2_unitree_legged_msgs/msg/high_state.h"
#include "ros2_unitree_legged_msgs/msg/motor_cmd.h"
#include "ros2_unitree_legged_msgs/msg/motor_state.h"
#include "ros2_unitree_legged_msgs/msg/bms_cmd.h"
#include "ros2_unitree_legged_msgs/msg/bms_state.h"
#include "ros2_unitree_legged_msgs/msg/imu.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "rclcpp/rclcpp.hpp"

UNITREE_LEGGED_SDK::BmsCmd rosMsg2Cmd(const ros2_unitree_legged_msgs::msg::BmsCmd &msg)
{
    UNITREE_LEGGED_SDK::BmsCmd cmd;

    cmd.off = msg.off;

    for (int i(0); i < 3; i++)
    {
        cmd.reserve[i] = msg.reserve[i];
    }

    return cmd;
}

UNITREE_LEGGED_SDK::HighCmd rosMsg2Cmd(const ros2_unitree_legged_msgs::msg::HighCmd::SharedPtr &msg)
{
    UNITREE_LEGGED_SDK::HighCmd cmd;

    for (int i(0); i < 2; i++)
    {
        cmd.head[i] = msg->head[i];
        cmd.SN[i] = msg->sn[i];
        cmd.version[i] = msg->version[i];
        cmd.position[i] = msg->position[i];
        cmd.velocity[i] = msg->velocity[i];
    }

    for (int i(0); i < 3; i++)
    {
        cmd.euler[i] = msg->euler[i];
    }

    for (int i(0); i < 4; i++)
    {
        cmd.led[i].r = msg->led[i].r;
        cmd.led[i].g = msg->led[i].g;
        cmd.led[i].b = msg->led[i].b;
    }

    for (int i(0); i < 40; i++)
    {
        cmd.wirelessRemote[i] = msg->wireless_remote[i];
    }

    cmd.levelFlag = msg->level_flag;
    cmd.frameReserve = msg->frame_reserve;
    cmd.bandWidth = msg->band_width;
    cmd.mode = msg->mode;
    cmd.gaitType = msg->gait_type;
    cmd.speedLevel = msg->speed_level;
    cmd.footRaiseHeight = msg->foot_raise_height;
    cmd.bodyHeight = msg->body_height;
    cmd.yawSpeed = msg->yaw_speed;
    cmd.reserve = msg->reserve;
    cmd.crc = msg->crc;

    cmd.bms = rosMsg2Cmd(msg->bms);

    return cmd;
}

UNITREE_LEGGED_SDK::MotorCmd rosMsg2Cmd(const ros2_unitree_legged_msgs::msg::MotorCmd &msg)
{
    UNITREE_LEGGED_SDK::MotorCmd cmd;

    cmd.mode = msg.mode;
    cmd.q = msg.q;
    cmd.dq = msg.dq;
    cmd.tau = msg.tau;
    cmd.Kp = msg.kp;
    cmd.Kd = msg.kd;

    for (int i(0); i < 3; i++)
    {
        cmd.reserve[i] = msg.reserve[i];
    }

    return cmd;
}

UNITREE_LEGGED_SDK::LowCmd rosMsg2Cmd(const ros2_unitree_legged_msgs::msg::LowCmd::SharedPtr &msg)
{
    UNITREE_LEGGED_SDK::LowCmd cmd;

    for (int i(0); i < 2; i++)
    {
        cmd.head[i] = msg->head[i];
        cmd.SN[i] = msg->sn[i];
        cmd.version[i] = msg->version[i];
    }

    for (int i(0); i < 40; i++)
    {
        cmd.wirelessRemote[i] = msg->wireless_remote[i];
    }

    for (int i(0); i < 20; i++)
    {
        cmd.motorCmd[i] = rosMsg2Cmd(msg->motor_cmd[i]);
    }

    cmd.bms = rosMsg2Cmd(msg->bms);

    cmd.levelFlag = msg->level_flag;
    cmd.frameReserve = msg->frame_reserve;
    cmd.bandWidth = msg->band_width;
    cmd.reserve = msg->reserve;
    cmd.crc = msg->crc;

    return cmd;
}

ros2_unitree_legged_msgs::msg::MotorState state2rosMsg(UNITREE_LEGGED_SDK::MotorState &state)
{
    ros2_unitree_legged_msgs::msg::MotorState ros_msg;

    ros_msg.mode = state.mode;
    ros_msg.q = state.q;
    ros_msg.dq = state.dq;
    ros_msg.ddq = state.ddq;
    ros_msg.tau_est = state.tauEst;
    ros_msg.q_raw = state.q_raw;
    ros_msg.dq_raw = state.dq_raw;
    ros_msg.ddq_raw = state.ddq_raw;
    ros_msg.temperature = state.temperature;

    ros_msg.reserve[0] = state.reserve[0];
    ros_msg.reserve[1] = state.reserve[1];

    return ros_msg;
}

ros2_unitree_legged_msgs::msg::IMU state2rosMsg(UNITREE_LEGGED_SDK::IMU &state)
{
    ros2_unitree_legged_msgs::msg::IMU ros_msg;

    for (int i(0); i < 4; i++)
    {
        ros_msg.quaternion[i] = state.quaternion[i];
    }

    for (int i(0); i < 3; i++)
    {
        ros_msg.gyroscope[i] = state.gyroscope[i];
        ros_msg.accelerometer[i] = state.accelerometer[i];
        ros_msg.rpy[i] = state.rpy[i];
    }

    ros_msg.temperature = state.temperature;

    return ros_msg;
}

ros2_unitree_legged_msgs::msg::BmsState state2rosMsg(UNITREE_LEGGED_SDK::BmsState &state)
{
    ros2_unitree_legged_msgs::msg::BmsState ros_msg;

    for (int i(0); i < 2; i++)
    {
        ros_msg.bq_ntc[i] = state.BQ_NTC[i];
        ros_msg.mcu_ntc[i] = state.MCU_NTC[i];
    }

    for (int i(0); i < 10; i++)
    {
        ros_msg.cell_vol[i] = state.cell_vol[i];
    }

    ros_msg.version_h = state.version_h;
    ros_msg.version_l = state.version_l;
    ros_msg.bms_status = state.bms_status;
    ros_msg.soc = state.SOC;
    ros_msg.current = state.current;
    ros_msg.cycle = state.cycle;

    return ros_msg;
}

ros2_unitree_legged_msgs::msg::LowState state2rosMsg(UNITREE_LEGGED_SDK::LowState &state)
{
    ros2_unitree_legged_msgs::msg::LowState ros_msg;

    for (int i(0); i < 2; i++)
    {
        ros_msg.head[i] = state.head[i];
        ros_msg.sn[i] = state.SN[i];
        ros_msg.version[i] = state.version[i];
    }

    for (int i(0); i < 4; i++)
    {
        ros_msg.foot_force[i] = state.footForce[i];
        ros_msg.foot_force_est[i] = state.footForceEst[i];
    }

    for (int i(0); i < 40; i++)
    {
        ros_msg.wireless_remote[i] = state.wirelessRemote[i];
    }

    for (int i(0); i < 20; i++)
    {
        ros_msg.motor_state[i] = state2rosMsg(state.motorState[i]);
    }

    ros_msg.imu = state2rosMsg(state.imu);

    ros_msg.bms = state2rosMsg(state.bms);

    ros_msg.tick = state.tick;
    ros_msg.reserve = state.reserve;
    ros_msg.crc = state.crc;

    return ros_msg;
}

ros2_unitree_legged_msgs::msg::Cartesian state2rosMsg(UNITREE_LEGGED_SDK::Cartesian &state)
{
    ros2_unitree_legged_msgs::msg::Cartesian ros_msg;

    ros_msg.x = state.x;
    ros_msg.y = state.y;
    ros_msg.z = state.z;

    return ros_msg;
}

ros2_unitree_legged_msgs::msg::HighState state2rosMsg(UNITREE_LEGGED_SDK::HighState &state)
{
    ros2_unitree_legged_msgs::msg::HighState ros_msg;

    for (int i(0); i < 2; i++)
    {
        ros_msg.head[i] = state.head[i];
        ros_msg.sn[i] = state.SN[i];
        ros_msg.version[i] = state.version[i];
    }

    for (int i(0); i < 4; i++)
    {
        ros_msg.foot_force[i] = state.footForce[i];
        ros_msg.foot_force_est[i] = state.footForceEst[i];
        ros_msg.range_obstacle[i] = state.rangeObstacle[i];
        ros_msg.foot_position2body[i] = state2rosMsg(state.footPosition2Body[i]);
        ros_msg.foot_speed2body[i] = state2rosMsg(state.footSpeed2Body[i]);
    }

    for (int i(0); i < 3; i++)
    {
        ros_msg.position[i] = state.position[i];
        ros_msg.velocity[i] = state.velocity[i];
    }

    for (int i(0); i < 40; i++)
    {
        ros_msg.wireless_remote[i] = state.wirelessRemote[i];
    }

    for (int i(0); i < 20; i++)
    {
        ros_msg.motor_state[i] = state2rosMsg(state.motorState[i]);
    }

    ros_msg.imu = state2rosMsg(state.imu);

    ros_msg.bms = state2rosMsg(state.bms);

    ros_msg.level_flag = state.levelFlag;
    ros_msg.frame_reserve = state.frameReserve;
    ros_msg.band_width = state.bandWidth;
    ros_msg.mode = state.mode;
    ros_msg.progress = state.progress;
    ros_msg.gait_type = state.gaitType;
    ros_msg.foot_raise_height = state.footRaiseHeight;
    ros_msg.body_height = state.bodyHeight;
    ros_msg.yaw_speed = state.yawSpeed;
    ros_msg.reserve = state.reserve;
    ros_msg.crc = state.crc;

    return ros_msg;
}

#endif // _CONVERT_H_