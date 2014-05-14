// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <cob_teleop_cob4/cob_teleop_cob4Config.h>

// ROS message includes
#include <sensor_msgs/JoyFeedback.h>
#include <geometry_msgs/Twist.h>
#include <brics_actuator/CartesianTwist.h>
#include <brics_actuator/CartesianTwist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <brics_actuator/JointVelocities.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

// other includes
#include <cob_teleop_cob4_common.cpp>


class cob_teleop_cob4_ros
{
    public:
    ros::NodeHandle n_;

    dynamic_reconfigure::Server<cob_teleop_cob4::cob_teleop_cob4Config> server;
    dynamic_reconfigure::Server<cob_teleop_cob4::cob_teleop_cob4Config>::CallbackType f;

    ros::Publisher joy_feedback_;
    ros::Publisher base_controller_command_;
    ros::Publisher arm_cart_left_;
    ros::Publisher arm_cart_right_;
    ros::Publisher arm_joint_right_;
    ros::Publisher arm_joint_left_;
    ros::Publisher head_controller_command_;
    ros::Publisher sensorring_controller_command_;
    ros::Publisher torso_controller_command_;
    ros::Subscriber joy_;

    cob_teleop_cob4_data component_data_;
    cob_teleop_cob4_config component_config_;
    cob_teleop_cob4_impl component_implementation_;

    cob_teleop_cob4_ros()
    {
        f = boost::bind(&cob_teleop_cob4_ros::configure_callback, this, _1, _2);
        server.setCallback(f);


        joy_feedback_ = n_.advertise<sensor_msgs::JoyFeedback>("joy_feedback", 1);
        base_controller_command_ = n_.advertise<geometry_msgs::Twist>("base_controller_command", 1);
        arm_cart_left_ = n_.advertise<brics_actuator::CartesianTwist>("arm_cart_left", 1);
        arm_cart_right_ = n_.advertise<brics_actuator::CartesianTwist>("arm_cart_right", 1);
        arm_joint_right_ = n_.advertise<trajectory_msgs::JointTrajectory>("arm_joint_right", 1);
        arm_joint_left_ = n_.advertise<trajectory_msgs::JointTrajectory>("arm_joint_left", 1);
        head_controller_command_ = n_.advertise<trajectory_msgs::JointTrajectory>("head_controller_command", 1);
        sensorring_controller_command_ = n_.advertise<brics_actuator::JointVelocities>("sensorring_controller_command", 1);
        torso_controller_command_ = n_.advertise<geometry_msgs::Twist>("torso_controller_command", 1);
        joy_ = n_.subscribe("joy", 1, &cob_teleop_cob4_ros::topicCallback_joy, this);

        n_.param("button_deadman", component_config_.button_deadman, (int)11);
        n_.param("base_max_linear", component_config_.base_max_linear, (double)2.0);
        n_.param("base_max_angular", component_config_.base_max_angular, (double)1.5);
        n_.param("torso_max_angular", component_config_.torso_max_angular, (double)0.1);
        n_.param("head_max_angular", component_config_.head_max_angular, (double)0.1);
        n_.param("sensor_ring_max_angular", component_config_.sensor_ring_max_angular, (double)0.1);
        n_.param("arm_joint_velocity_max", component_config_.arm_joint_velocity_max, (double)0.1);
        n_.param("arm_cartesian_max_linear", component_config_.arm_cartesian_max_linear, (double)0.1);
        n_.param("arm_cartesian_max_angular", component_config_.arm_cartesian_max_angular, (double)0.1);
        n_.param("gripper_max_velocity", component_config_.gripper_max_velocity, (double)0.1);
        n_.param("base_x", component_config_.base_x, (int)1);
        n_.param("base_y", component_config_.base_y, (int)0);
        n_.param("base_yaw", component_config_.base_yaw, (int)2);
        n_.param("arm_x", component_config_.arm_x, (int)0);
        n_.param("arm_y", component_config_.arm_y, (int)1);
        n_.param("arm_yaw", component_config_.arm_yaw, (int)2);
        n_.param("arm_pitch_up", component_config_.arm_pitch_up, (int)4);
        n_.param("arm_pitch_down", component_config_.arm_pitch_down, (int)6);
        n_.param("arm_roll_right_and_ellbow", component_config_.arm_roll_right_and_ellbow, (int)5);
        n_.param("arm_roll_left_and_ellbow", component_config_.arm_roll_left_and_ellbow, (int)7);
        n_.param("arm_z_up", component_config_.arm_z_up, (int)12);
        n_.param("arm_z_down", component_config_.arm_z_down, (int)14);
        n_.param("gripper_open", component_config_.gripper_open, (int)15);
        n_.param("gripper_close", component_config_.gripper_close, (int)13);
        n_.param("arm_joint_up", component_config_.arm_joint_up, (int)4);
        n_.param("arm_joint_down", component_config_.arm_joint_down, (int)6);
        n_.param("arm_joint_left", component_config_.arm_joint_left, (int)7);
        n_.param("arm_joint_right", component_config_.arm_joint_right, (int)5);
        n_.param("arm_joint_12", component_config_.arm_joint_12, (int)15);
        n_.param("arm_joint_34", component_config_.arm_joint_34, (int)14);
        n_.param("arm_joint_56", component_config_.arm_joint_56, (int)13);
        n_.param("arm_joint_7_gripper", component_config_.arm_joint_7_gripper, (int)12);
        n_.param("axis_runfactor", component_config_.axis_runfactor, (int)9);
        n_.param("button_safety_override", component_config_.button_safety_override, (int)9);
        n_.param("button_init_recover", component_config_.button_init_recover, (int)3);
        n_.param("button_mode_switch", component_config_.button_mode_switch, (int)0);
        n_.param("torso_roll", component_config_.torso_roll, (int)0);
        n_.param("torso_pitch", component_config_.torso_pitch, (int)1);
        n_.param("torso_yaw_left", component_config_.torso_yaw_left, (int)15);
        n_.param("torso_yaw_right", component_config_.torso_yaw_right, (int)13);
        n_.param("sensorring_yaw_left", component_config_.sensorring_yaw_left, (int)4);
        n_.param("sensorring_yaw_right", component_config_.sensorring_yaw_right, (int)6);
        }

    void topicCallback_joy(const sensor_msgs::Joy::ConstPtr& msg)
    {
        component_data_.in_joy = *msg;
    }

    void configure_callback(cob_teleop_cob4::cob_teleop_cob4Config &config, uint32_t level)
    {
        component_config_.button_deadman = config.button_deadman;
        component_config_.base_max_linear = config.base_max_linear;
        component_config_.base_max_angular = config.base_max_angular;
        component_config_.torso_max_angular = config.torso_max_angular;
        component_config_.head_max_angular = config.head_max_angular;
        component_config_.sensor_ring_max_angular = config.sensor_ring_max_angular;
        component_config_.arm_joint_velocity_max = config.arm_joint_velocity_max;
        component_config_.arm_cartesian_max_linear = config.arm_cartesian_max_linear;
        component_config_.arm_cartesian_max_angular = config.arm_cartesian_max_angular;
        component_config_.gripper_max_velocity = config.gripper_max_velocity;
        component_config_.base_x = config.base_x;
        component_config_.base_y = config.base_y;
        component_config_.base_yaw = config.base_yaw;
        component_config_.arm_x = config.arm_x;
        component_config_.arm_y = config.arm_y;
        component_config_.arm_yaw = config.arm_yaw;
        component_config_.arm_pitch_up = config.arm_pitch_up;
        component_config_.arm_pitch_down = config.arm_pitch_down;
        component_config_.arm_roll_right_and_ellbow = config.arm_roll_right_and_ellbow;
        component_config_.arm_roll_left_and_ellbow = config.arm_roll_left_and_ellbow;
        component_config_.arm_z_up = config.arm_z_up;
        component_config_.arm_z_down = config.arm_z_down;
        component_config_.gripper_open = config.gripper_open;
        component_config_.gripper_close = config.gripper_close;
        component_config_.arm_joint_up = config.arm_joint_up;
        component_config_.arm_joint_down = config.arm_joint_down;
        component_config_.arm_joint_left = config.arm_joint_left;
        component_config_.arm_joint_right = config.arm_joint_right;
        component_config_.arm_joint_12 = config.arm_joint_12;
        component_config_.arm_joint_34 = config.arm_joint_34;
        component_config_.arm_joint_56 = config.arm_joint_56;
        component_config_.arm_joint_7_gripper = config.arm_joint_7_gripper;
        component_config_.axis_runfactor = config.axis_runfactor;
        component_config_.button_safety_override = config.button_safety_override;
        component_config_.button_init_recover = config.button_init_recover;
        component_config_.button_mode_switch = config.button_mode_switch;
        component_config_.torso_roll = config.torso_roll;
        component_config_.torso_pitch = config.torso_pitch;
        component_config_.torso_yaw_left = config.torso_yaw_left;
        component_config_.torso_yaw_right = config.torso_yaw_right;
        component_config_.sensorring_yaw_left = config.sensorring_yaw_left;
        component_config_.sensorring_yaw_right = config.sensorring_yaw_right;
    }

    void configure()
    {
        component_implementation_.configure(component_config_);
    }

    void activate_all_output()
    {
        component_data_.out_joy_feedback_active = true;
        component_data_.out_base_controller_command_active = true;
        component_data_.out_arm_cart_left_active = true;
        component_data_.out_arm_cart_right_active = true;
        component_data_.out_arm_joint_right_active = true;
        component_data_.out_arm_joint_left_active = true;
        component_data_.out_head_controller_command_active = true;
        component_data_.out_sensorring_controller_command_active = true;
        component_data_.out_torso_controller_command_active = true;
    }

    void update()
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
        if (component_data_.out_joy_feedback_active)
            joy_feedback_.publish(component_data_.out_joy_feedback);
        if (component_data_.out_base_controller_command_active)
            base_controller_command_.publish(component_data_.out_base_controller_command);
        if (component_data_.out_arm_cart_left_active)
            arm_cart_left_.publish(component_data_.out_arm_cart_left);
        if (component_data_.out_arm_cart_right_active)
            arm_cart_right_.publish(component_data_.out_arm_cart_right);
        if (component_data_.out_arm_joint_right_active)
            arm_joint_right_.publish(component_data_.out_arm_joint_right);
        if (component_data_.out_arm_joint_left_active)
            arm_joint_left_.publish(component_data_.out_arm_joint_left);
        if (component_data_.out_head_controller_command_active)
            head_controller_command_.publish(component_data_.out_head_controller_command);
        if (component_data_.out_sensorring_controller_command_active)
            sensorring_controller_command_.publish(component_data_.out_sensorring_controller_command);
        if (component_data_.out_torso_controller_command_active)
            torso_controller_command_.publish(component_data_.out_torso_controller_command);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "cob_teleop_cob4");

    cob_teleop_cob4_ros node;
    node.configure();

    ros::Rate loop_rate(40.0);

    while(node.n_.ok())
    {
        node.update();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
