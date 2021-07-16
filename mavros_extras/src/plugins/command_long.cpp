/**
 * @brief Command Long messages plugin
 * @file command_long.cpp
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

#include <mavros_msgs/CommandLongMSG.h>

namespace mavros
{
    namespace extra_plugins
    {

        using utils::enum_value;
        class CommandLongPlugin : public plugin::PluginBase
        {
        public:
            CommandLongPlugin() : PluginBase(),
                                  cmd_nh("~cmd_long"),
                                  use_comp_id_system_control(false)
            {
            }

            void initialize(UAS &uas_) override
            {
                PluginBase::initialize(uas_);

                cmd_nh.param("use_comp_id_system_control", use_comp_id_system_control, false);

                command_sub = cmd_nh.subscribe("send", 10, &CommandLongPlugin::command_long_cb, this);
                command_pub = cmd_nh.advertise<mavros_msgs::CommandLongMSG>("command", 10);
            }

            Subscriptions get_subscriptions() override
            {
                return {make_handler(&CommandLongPlugin::handle_command)};
            }

        private:
            ros::NodeHandle cmd_nh;
            ros::Subscriber command_sub;
            ros::Publisher command_pub;

            bool use_comp_id_system_control;

            void handle_command(const mavlink::mavlink_message_t *msg,
                                mavlink::common::msg::COMMAND_LONG &cmd_long)
            {
                auto cl_msg = boost::make_shared<mavros_msgs::CommandLongMSG>();
                cl_msg->param1 = cmd_long.param1;
                cl_msg->param2 = cmd_long.param2;
                cl_msg->param3 = cmd_long.param3;
                cl_msg->param4 = cmd_long.param4;
                cl_msg->param5 = cmd_long.param5;
                cl_msg->param6 = cmd_long.param6;
                cl_msg->param7 = cmd_long.param7;
                cl_msg->command = cmd_long.command;

                command_pub.publish(cl_msg);
            }

            void command_long_cb(const mavros_msgs::CommandLongMSG::ConstPtr &req)
            {
                mavlink::common::msg::COMMAND_LONG cmd{};
                set_target(cmd);

                cmd.confirmation = 0;
                cmd.command = req->command;
                cmd.param1 = req->param1;
                cmd.param2 = req->param2;
                cmd.param3 = req->param3;
                cmd.param4 = req->param4;
                cmd.param5 = req->param5;
                cmd.param6 = req->param6;
                cmd.param7 = req->param7;

                UAS_FCU(m_uas)->send_message_ignore_drop(cmd);
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
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::CommandLongPlugin, mavros::plugin::PluginBase)