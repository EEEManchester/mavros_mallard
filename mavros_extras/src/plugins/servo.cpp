/**
 * @brief Command Long messages plugin
 * @file servo.cpp
 * @author Zhengyi Jiang <anthony.zyjiang@outlook.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2021 Zhengyi Jiang.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/Servo.h>

#define MAV_CMD_ID_DO_SET_SERVO 183

namespace mavros
{
    namespace extra_plugins
    {

        using utils::enum_value;
        class ServoPlugin : public plugin::PluginBase
        {
        public:
            ServoPlugin() : PluginBase(),
                                  cmd_nh("~servo"),
                                  use_comp_id_system_control(false)
            {
            }

            void initialize(UAS &uas_) override
            {
                PluginBase::initialize(uas_);

                cmd_nh.param("use_comp_id_system_control", use_comp_id_system_control, false);

                servo_sub = cmd_nh.subscribe("send", 10, &ServoPlugin::servo_cb, this);
                servo_pub = cmd_nh.advertise<mavros_msgs::Servo>("servo", 10);
            }

            Subscriptions get_subscriptions() override
            {
                return {make_handler(&ServoPlugin::handle_servo)};
            }

        private:
            ros::NodeHandle cmd_nh;
            ros::Subscriber servo_sub;
            ros::Publisher servo_pub;

            bool use_comp_id_system_control;

            void handle_servo(const mavlink::mavlink_message_t *msg,
                                mavlink::common::msg::COMMAND_LONG &cmd_long)
            {
                auto servo_msg = boost::make_shared<mavros_msgs::Servo>();
                servo_msg->servo_id = cmd_long.param1;
                servo_msg->pwm = cmd_long.param2;

                servo_pub.publish(servo_msg);
            }

            void servo_cb(const mavros_msgs::Servo::ConstPtr &req)
            {
                mavlink::common::msg::COMMAND_LONG cmd_long{};
                set_target(cmd_long);

                cmd_long.param1 = req->servo_id;
                cmd_long.param2 = req->pwm;
                cmd_long.command = MAV_CMD_ID_DO_SET_SERVO;

                UAS_FCU(m_uas)->send_message_ignore_drop(cmd_long);
            }

            template <typename MsgT>
            inline void set_target(MsgT &cmd)
            {
                using mavlink::minimal::MAV_COMPONENT;

                const uint8_t tgt_sys_id = m_uas->get_tgt_system();
                const uint8_t tgt_comp_id = (use_comp_id_system_control) ? enum_value(MAV_COMPONENT::COMP_ID_SYSTEM_CONTROL)
                                                                         : m_uas->get_tgt_component();

                cmd.target_system = tgt_sys_id;
                cmd.target_component = tgt_comp_id;
            }
        };
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ServoPlugin, mavros::plugin::PluginBase)