#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

// Message types
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
//#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "actionlib_msgs/msg/goal_status.hpp"

// TF2 (Transform listener)
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

// Action messages (used less directly in ROS 2; here for compatibility)
#include "action_msgs/msg/goal_status.hpp"

// Standard
#include <iostream>
#include <vector>
#include <exception>
///

#define RATE 30

#define SM_INIT 0
#define SM_WAITING_FOR_TASK 11
#define SM_GOAL_POSE_ACCEL 1
#define SM_GOAL_POSE_CRUISE 2
#define SM_GOAL_POSE_DECCEL 3
#define SM_GOAL_POSE_CORRECT_ANGLE 4
#define SM_GOAL_POSE_FINISH 10
#define SM_GOAL_POSE_FAILED 12
#define SM_GOAL_PATH_ACCEL 5
#define SM_GOAL_PATH_CRUISE 6
#define SM_GOAL_PATH_DECCEL 7
#define SM_GOAL_PATH_FINISH 8
#define SM_GOAL_PATH_FAILED 81
#define SM_COLLISION_RISK 9

class SimpleMoveNode : public rclcpp::Node
{
public:
    SimpleMoveNode() : Node("simple_move_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        //############
        // Declare parameters with default values
        this->declare_parameter<bool>("use_namespace",          false);
        this->declare_parameter<float>("max_linear_speed",      0.3f);
        this->declare_parameter<float>("min_linear_speed",      0.05f);
        this->declare_parameter<float>("max_angular_speed",     1.0f);
        this->declare_parameter<float>("control_alpha",         0.6548f);
        this->declare_parameter<float>("control_beta",          0.2f);
        this->declare_parameter<float>("linear_acceleration",   0.1f);
        this->declare_parameter<float>("fine_dist_tolerance",   0.03f);
        this->declare_parameter<float>("coarse_dist_tolerance", 0.2f);
        this->declare_parameter<float>("angle_tolerance",       0.05f);

        this->declare_parameter<bool>("move_head", true);
        this->declare_parameter<bool>("use_pot_fields", false);

        this->declare_parameter<std::string>("base_link_name", "base_footprint");
        this->declare_parameter<std::string>("odom_name", "odom");

        // Initialize internal variables from declared parameters
        this->get_parameter("use_namespace",         use_namespace_);
        this->get_parameter("max_linear_speed",      max_linear_speed_);
        this->get_parameter("min_linear_speed",      min_linear_speed_);
        this->get_parameter("max_angular_speed",     max_angular_speed_);
        this->get_parameter("control_alpha",         alpha_);
        this->get_parameter("control_beta",          beta_);
        this->get_parameter("linear_acceleration",   linear_acceleration_);
        this->get_parameter("fine_dist_tolerance",   fine_dist_tolerance_);
        this->get_parameter("coarse_dist_tolerance", coarse_dist_tolerance_);
        this->get_parameter("angle_tolerance",       angle_tolerance_);

        this->get_parameter("move_head",             move_head_);
        this->get_parameter("use_pot_fields",        use_pot_fields_);

        this->get_parameter("base_link_name",        base_link_name_);
        this->get_parameter("odom_name",             odom_name_);

        // Setup parameter change callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&SimpleMoveNode::on_parameter_change, this, std::placeholders::_1));

        //############
        // Publishers
        pub_cmd_vel_        = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 
            rclcpp::QoS(10).transient_local());
        pub_goal_reached_   = this->create_publisher<actionlib_msgs::msg::GoalStatus>(
            make_name("/simple_move/goal_reached"), 
            rclcpp::QoS(10).transient_local());
        pub_head_goal_pose_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            make_name("/hardware/head/goal_pose"), 
            rclcpp::QoS(10).transient_local());

        //############
        // Subscribers
        //// stop
        sub_generalStop_ = this->create_subscription<std_msgs::msg::Empty>(
            make_name("/stop"), 
            rclcpp::SensorDataQoS(), 
            std::bind(&SimpleMoveNode::callback_general_stop, this, std::placeholders::_1));

        sub_navCtrlStop_ = this->create_subscription<std_msgs::msg::Empty>(
            make_name("/navigation/stop"), 
            rclcpp::SensorDataQoS(), 
            std::bind(&SimpleMoveNode::callback_navigation_stop, this, std::placeholders::_1));

        sub_navSimpleMvStop_ = this->create_subscription<std_msgs::msg::Empty>(
            make_name("/simple_move/stop"), 
            rclcpp::SensorDataQoS(), 
            std::bind(&SimpleMoveNode::callback_simple_move_stop, this, std::placeholders::_1));

        //// goal
        sub_goalDistance_ = this->create_subscription<std_msgs::msg::Float32>(
            make_name("/simple_move/goal_dist"), 
            rclcpp::SensorDataQoS(), 
            std::bind(&SimpleMoveNode::callback_goal_dist, this, std::placeholders::_1));
        
        sub_goalDistAngle_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            make_name("/simple_move/goal_dist_angle"), 
            rclcpp::SensorDataQoS(), 
            std::bind(&SimpleMoveNode::callback_goal_dist_angle, this, std::placeholders::_1));

        sub_goalPath_ = this->create_subscription<nav_msgs::msg::Path>(
            make_name("/simple_move/goal_path"), 
            rclcpp::SensorDataQoS(), 
            std::bind(&SimpleMoveNode::callback_goal_path, this, std::placeholders::_1));

        //// motion
        sub_moveLateral_ = this->create_subscription<std_msgs::msg::Float32>(
            make_name("/simple_move/goal_dist_lateral"), 
            rclcpp::SensorDataQoS(), 
            std::bind(&SimpleMoveNode::callback_move_lateral, this, std::placeholders::_1));

        sub_collisionRisk_ = this->create_subscription<std_msgs::msg::Bool>(
            make_name("/navigation/potential_fields/collision_risk"), 
            rclcpp::SensorDataQoS(), 
            std::bind(&SimpleMoveNode::callback_collision_risk, this, std::placeholders::_1));

        sub_rejectionForce_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            make_name("/navigation/potential_fields/pf_rejection_force"), 
            rclcpp::SensorDataQoS(), 
            std::bind(&SimpleMoveNode::callback_rejection_force, this, std::placeholders::_1));
        
        //############
        // Wait for transforms
        wait_for_transforms("map", base_link_name_);
        wait_for_transforms(odom_name_, base_link_name_);

        // Simple Move main processing
        processing_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&SimpleMoveNode::simple_move_processing, this));

        RCLCPP_INFO(this->get_logger(), "SimpleMove.-> SimpleMoveNode is ready.");
    }

private:
    //############
    // State variables
    bool use_namespace_   = false;
    float goal_distance_  = 0;
    float goal_angle_     = 0;
    bool  new_pose_       = false;
    bool  new_path_       = false;
    bool  collision_risk_ = false;
    bool  move_lat_       = false;
    bool  stop_           = false;

    float robot_x_ = 0;
    float robot_y_ = 0;
    float robot_t_ = 0;
    float goal_x_ = 0;
    float goal_y_ = 0;
    float goal_t_ = 0;
    float global_goal_x_ = 0;
    float global_goal_y_ = 0;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    nav_msgs::msg::Path           goal_path_;
    geometry_msgs::msg::Vector3   rejection_force_;

    // Internal parameter values
    float max_linear_speed_;
    float min_linear_speed_;
    float max_angular_speed_;
    float alpha_;
    float beta_;
    float linear_acceleration_;
    float fine_dist_tolerance_;
    float coarse_dist_tolerance_;
    float angle_tolerance_;

    bool move_head_;
    bool use_pot_fields_;
    std::string base_link_name_;
    std::string odom_name_;

    // Simple Move processing variables
    actionlib_msgs::msg::GoalStatus msg_goal_reached;

    int state = SM_INIT;
    float current_linear_speed = 0;
    int prev_pose_idx = 0;
    int next_pose_idx = 0;
    float temp_k = 0;
    int attempts = 0;
    float global_error = 0;


    //############
    // Publishers
    rclcpp::Publisher<actionlib_msgs::msg::GoalStatus>::SharedPtr   pub_goal_reached_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr         pub_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr  pub_head_goal_pose_;


    //############
    // Subscribers
    //// stop
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_generalStop_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_navCtrlStop_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_navSimpleMvStop_;

    //// goal
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_goalDistance_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_goalDistAngle_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_goalPath_;

    //// motion
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_moveLateral_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_collisionRisk_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_rejectionForce_;


    //############
    // Parameter callback handle
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Main processing loop
    rclcpp::TimerBase::SharedPtr processing_timer_;


    //############
    // Runtime parameter update callback
    rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : params)
        {
            if (param.get_name()      == "use_namespace")         use_namespace_           = param.as_bool();
            else if (param.get_name() == "max_linear_speed")      max_linear_speed_        = param.as_double();
            else if (param.get_name() == "min_linear_speed")      min_linear_speed_        = param.as_double();
            else if (param.get_name() == "max_angular_speed")     max_angular_speed_       = param.as_double();
            else if (param.get_name() == "control_alpha")         alpha_                   = param.as_double();
            else if (param.get_name() == "control_beta")          beta_                    = param.as_double();
            else if (param.get_name() == "linear_acceleration")   linear_acceleration_     = param.as_double();
            else if (param.get_name() == "fine_dist_tolerance")   fine_dist_tolerance_     = param.as_double();
            else if (param.get_name() == "coarse_dist_tolerance") coarse_dist_tolerance_   = param.as_double();
            else if (param.get_name() == "angle_tolerance")       angle_tolerance_         = param.as_double();

            else if (param.get_name() == "move_head")             move_head_               = param.as_bool();
            else if (param.get_name() == "use_pot_fields")        use_pot_fields_          = param.as_bool();

            else if (param.get_name() == "base_link_name")        base_link_name_          = param.as_string();
            else if (param.get_name() == "odom_name")             odom_name_               = param.as_string();

            else {
                result.successful = false;
                result.reason = "SimpleMove.-> Unsupported parameter: " + param.get_name();
                RCLCPP_WARN(this->get_logger(), "SimpleMove.-> Attempted to update unsupported parameter: %s", param.get_name().c_str());
                break;
            }
        }

        return result;
    }

    std::string make_name(const std::string &suffix) const
    {
        // Ensure suffix starts with "/"
        std::string sfx = suffix;
        if (!sfx.empty() && sfx.front() != '/')
            sfx = "/" + sfx;

        std::string name;

        if (use_namespace_) {
            // Use node namespace prefix
            name = this->get_namespace() + sfx;

            // Avoid accidental double slash (e.g., when namespace is "/")
            if (name.size() > 1 && name[0] == '/' && name[1] == '/')
                name.erase(0, 1);
        } else {
            // Use global namespace (no node namespace prefix)
            name = sfx;
        }

        return name;
    }

    // Wait for transforms 
    void wait_for_transforms(const std::string &target_frame, const std::string &source_frame)
    {
        RCLCPP_INFO(this->get_logger(),
                    "SimpleMove.-> Waiting for transform from '%s' to '%s'...", source_frame.c_str(), target_frame.c_str());

        rclcpp::Time start_time = this->now();
        rclcpp::Duration timeout = rclcpp::Duration::from_seconds(10.0); 

        bool transform_ok = false;

        while (rclcpp::ok() && (this->now() - start_time) < timeout) {
            try {
                tf_buffer_.lookupTransform(
                    target_frame,
                    source_frame,
                    tf2::TimePointZero,
                    tf2::durationFromSec(0.1)
                );
                transform_ok = true;
                break;
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                    "SimpleMove.-> Still waiting for transform: %s", ex.what());
            }

            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }

        if (!transform_ok) {
            RCLCPP_WARN(this->get_logger(),
                        "SimpleMove.-> Timeout while waiting for transform from '%s' to '%s'.",
                        source_frame.c_str(), target_frame.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "SimpleMove.-> Transform from '%s' to '%s' is now available.",
                        source_frame.c_str(), target_frame.c_str());
        }
    }


    //############
    //Simple Move callbacks
    void callback_general_stop(const std_msgs::msg::Empty::SharedPtr msg) 
    {
        try 
        {
            std::cout << "SimpleMove.-> General Stop signal received " << std::endl;
            stop_     = true;
            new_pose_ = false;
            new_path_ = false;
            move_lat_ = false;
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "SimpleMove.-> Error processing callback_general_stop: %s", e.what());
        }
    }

    void callback_navigation_stop(const std_msgs::msg::Empty::SharedPtr msg) 
    {
        try 
        {
            std::cout << "SimpleMove.-> Navigation Stop signal received" << std::endl;
            stop_     = true;
            new_pose_ = false;
            new_path_ = false;
            move_lat_ = false;
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "SimpleMove.-> Error processing callback_navigation_stop: %s", e.what());
        }
    }

    void callback_simple_move_stop(const std_msgs::msg::Empty::SharedPtr msg) 
    {
        try 
        {
            std::cout << "SimpleMove.-> Simple move stop signal received" << std::endl;
            stop_     = true;
            new_pose_ = false;
            new_path_ = false;
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "SimpleMove.-> Error processing callback_simple_move_stop: %s", e.what());
        }
    }

    void callback_goal_dist(const std_msgs::msg::Float32::SharedPtr msg) 
    {
        try 
        {
            std::cout << "SimpleMove.-> New move received: goal dist= " << msg->data << std::endl;     
            goal_distance_ = msg->data;
            goal_angle_    = 0;
            new_pose_ = true;
            new_path_ = false;
            move_lat_ = false;
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "SimpleMove.-> Error processing callback_goal_dist: %s", e.what());
        }
    }

    void callback_goal_dist_angle(const std_msgs::msg::Float32MultiArray::SharedPtr msg) 
    {
        try 
        {
            std::cout << "SimpleMove.-> New move received: goal dist= " << msg->data[0] << " and goal angle= " << msg->data[1] << std::endl;
            goal_distance_ = msg->data[0];
            goal_angle_    = msg->data[1];
            new_pose_ = true;
            new_path_ = false;
            move_lat_ = false;
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "SimpleMove.-> Error processing callback_goal_dist_angle: %s", e.what());
        }
    }

    void callback_goal_path(const nav_msgs::msg::Path::SharedPtr msg) 
    {
        try 
        {
            std::cout << "SimpleMove.-> New path received with " << msg->poses.size()
                    << " points. Stamp= " << msg->header.stamp.sec << "s, Frame="  << msg->header.frame_id
                    << std::endl;
            move_lat_ = false;
            if (msg->poses.size() <= 0)
            {
                new_pose_ = false;
                new_path_ = false;   
            }else{
                goal_path_ = *msg;
                new_pose_ = false;
                new_path_ = true;
            }
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "SimpleMove.-> Error processing callback_goal_path: %s", e.what());
        }
    }

    void callback_move_lateral(const std_msgs::msg::Float32::SharedPtr msg) 
    {
        try 
        {
            goal_distance_ = msg->data;
            goal_angle_    = 0;
            new_pose_ = true;
            new_path_ = false;
            move_lat_ = true;
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "SimpleMove.-> Error processing callback_move_lateral: %s", e.what());
        }
    }

    void callback_collision_risk(const std_msgs::msg::Bool::SharedPtr msg) 
    {
        try 
        {
            collision_risk_ = msg->data;
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "SimpleMove.-> Error processing callback_collision_risk: %s", e.what());
        }
    }

    void callback_rejection_force(const geometry_msgs::msg::Vector3::SharedPtr msg) 
    {
        try 
        {
            rejection_force_ = *msg;
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "SimpleMove.-> Error processing callback_rejection_force: %s", e.what());
        }
    }


    //############
    //Simple Move additional functions
    geometry_msgs::msg::Twist calculate_speeds( float robot_x, float robot_y, float robot_t,
                                                float goal_x, float goal_y,
                                                float min_linear_speed, float max_linear_speed, float angular_speed, 
                                                float alpha, float beta,
                                                bool backwards, bool move_lat, 
                                                bool use_pot_fields=false,  double rejection_force_y=0.0)
    {
        float angle_error = 0;
        if(backwards) angle_error = (atan2(robot_y - goal_y, robot_x -goal_x)-robot_t);
        else angle_error = (atan2(goal_y - robot_y, goal_x - robot_x)-robot_t);
        if(angle_error >   M_PI) angle_error -= 2*M_PI;
        if(angle_error <= -M_PI) angle_error += 2*M_PI;
        if(move_lat) angle_error -= M_PI/2;
        if(angle_error <= -M_PI) angle_error += 2*M_PI;
        if(backwards) max_linear_speed *= -1;
        
        geometry_msgs::msg::Twist result;
        if(move_lat)
            result.linear.y   = max_linear_speed  * exp(-(angle_error * angle_error) / (alpha));
        else result.linear.x  = max_linear_speed  * exp(-(angle_error * angle_error) / (alpha));
        result.angular.z = angular_speed * (2 / (1 + exp(-angle_error / beta)) - 1);

        if(fabs(result.linear.x) < min_linear_speed)
            result.linear.x = 0;
        if(fabs(result.linear.y) < min_linear_speed)
            result.linear.y = 0;
        
        if(use_pot_fields && !move_lat) result.linear.y = rejection_force_y;
        
        return result;
    }

    geometry_msgs::msg::Twist calculate_speeds( float robot_angle, float goal_angle, float angular_speed, float beta)
    {
        float angle_error = goal_angle - robot_angle;
        if(angle_error >   M_PI) angle_error -= 2*M_PI;
        if(angle_error <= -M_PI) angle_error += 2*M_PI;

        geometry_msgs::msg::Twist result;
        result.linear.x  = 0;
        result.angular.z = angular_speed * (2 / (1 + exp(-angle_error / beta)) - 1);
        
        return result;
    }

    bool get_robot_position_wrt_map()
    {
        try
        {
            geometry_msgs::msg::TransformStamped transformStamped =
                tf_buffer_.lookupTransform("map", base_link_name_, tf2::TimePointZero);

            robot_x_ = transformStamped.transform.translation.x;
            robot_y_ = transformStamped.transform.translation.y;

            tf2::Quaternion q(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);

            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            robot_t_ = static_cast<float>(yaw);

            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            //RCLCPP_WARN(this->get_logger(), "SimpleMove.-> TF Exception: %s", ex.what());
            return false;
        }
    }

    bool get_robot_position_wrt_odom()
    {
        try
        {
            geometry_msgs::msg::TransformStamped transformStamped =
                tf_buffer_.lookupTransform(odom_name_, base_link_name_, tf2::TimePointZero);

            robot_x_ = transformStamped.transform.translation.x;
            robot_y_ = transformStamped.transform.translation.y;

            tf2::Quaternion q(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);

            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            robot_t_ = static_cast<float>(yaw);

            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            //RCLCPP_WARN(this->get_logger(), "SimpleMove.-> TF Exception: %s", ex.what());
            return false;
        }
    }

    bool get_goal_position_wrt_odom()
    {
        try
        {
            geometry_msgs::msg::TransformStamped transformStamped =
                tf_buffer_.lookupTransform(odom_name_, base_link_name_, tf2::TimePointZero);

            float robot_x = transformStamped.transform.translation.x;
            float robot_y = transformStamped.transform.translation.y;

            tf2::Quaternion q(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);

            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            float robot_t = static_cast<float>(yaw);

            goal_t_ = robot_t + goal_angle_;
            if(goal_t_ >   M_PI) goal_t_ -= 2*M_PI;
            if(goal_t_ <= -M_PI) goal_t_ += 2*M_PI;
            
            goal_x_ = robot_x + goal_distance_ * cos(robot_t + goal_angle_ + (move_lat_?M_PI/2:0));
            goal_y_ = robot_y + goal_distance_ * sin(robot_t + goal_angle_ + (move_lat_?M_PI/2:0));

            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            //RCLCPP_WARN(this->get_logger(), "SimpleMove.-> TF Exception: %s", ex.what());
            return false;
        }
    }
    
    float get_path_total_distance(nav_msgs::msg::Path& path)
    {
        float dist = 0;
        for(size_t i=1; i < path.poses.size(); i++)
            dist += sqrt(pow(path.poses[i].pose.position.x - path.poses[i-1].pose.position.x,2) +
                        pow(path.poses[i].pose.position.y - path.poses[i-1].pose.position.y,2));
        return dist;
    }

    void get_next_goal_from_path(int& last_pose_idx, int& next_pose_idx)
    {
        if(next_pose_idx >= (int)goal_path_.poses.size()) next_pose_idx = (int)goal_path_.poses.size() - 1;
        float prev_goal_x = goal_path_.poses[last_pose_idx].pose.position.x;
        float prev_goal_y = goal_path_.poses[last_pose_idx].pose.position.y;
        float robot_error_x = robot_x_ - prev_goal_x;
        float robot_error_y = robot_y_ - prev_goal_y;
        float error = 0;
        do
        {
            goal_x_ = goal_path_.poses[next_pose_idx].pose.position.x;
            goal_y_ = goal_path_.poses[next_pose_idx].pose.position.y;
            float path_vector_x = goal_x_ - prev_goal_x;
            float path_vector_y = goal_y_ - prev_goal_y;
            float path_vector_mag = sqrt(path_vector_x*path_vector_x + path_vector_y*path_vector_y);
            path_vector_x = path_vector_mag > 0 ? path_vector_x / path_vector_mag : 0;
            path_vector_y = path_vector_mag > 0 ? path_vector_y / path_vector_mag : 0;
            float scalar_projection = robot_error_x*path_vector_x + robot_error_y*path_vector_y;
            //Error is not euclidean distance, but it is measured projected on the current path segment
            error = path_vector_mag - scalar_projection;
            if(error < 0.25)
                last_pose_idx = next_pose_idx;
        }while(error < 0.25 && ++next_pose_idx < (int)goal_path_.poses.size());
    }

    std_msgs::msg::Float32MultiArray get_next_goal_head_angles(int next_pose_idx)
    {
        std_msgs::msg::Float32MultiArray msg;
        int idx = next_pose_idx + 5 >=  (int)goal_path_.poses.size() - 1 ? goal_path_.poses.size() - 1 : next_pose_idx + 5;
        float goal_x = goal_path_.poses[idx].pose.position.x;
        float goal_y = goal_path_.poses[idx].pose.position.y;
        float angle = atan2(goal_y - robot_y_, goal_x - robot_x_) - robot_t_;
        if(angle >   M_PI) angle -= 2*M_PI;
        if(angle <= -M_PI) angle += 2*M_PI;
        msg.data.push_back(angle);
        msg.data.push_back(-1.0);
        return msg;
    }


    //############
    //Simple Move main processing
    void simple_move_processing() 
    {
        try
        {
            if(stop_)
            {
                stop_ = false;
                state = SM_INIT;
                collision_risk_ = false;
                msg_goal_reached.status = actionlib_msgs::msg::GoalStatus::ABORTED;
                pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
                pub_goal_reached_->publish(msg_goal_reached);
            }
            if(new_pose_ || new_path_)
                state = SM_WAITING_FOR_TASK;

            switch(state)
            {
            case SM_INIT:
                std::cout << "SimpleMove.-> Low level control ready. Waiting for new goal. " << std::endl;
                collision_risk_ = false;
                state = SM_WAITING_FOR_TASK;
                break;
            
            
            case SM_WAITING_FOR_TASK:
                if(new_pose_)
                {
                    collision_risk_ = false;
                    get_goal_position_wrt_odom();
                    state = SM_GOAL_POSE_ACCEL;
                    new_pose_ = false;
                    msg_goal_reached.goal_id.id = "-1";
                    attempts = (int)((fabs(goal_distance_)*5)/max_linear_speed_*RATE + fabs(goal_angle_*5)/max_angular_speed_*RATE + RATE*5);
                }
                if(new_path_)
                {
                    collision_risk_ = false;
                    state = SM_GOAL_PATH_ACCEL;
                    new_path_ = false;
                    prev_pose_idx = 0;
                    next_pose_idx = 0;
                    global_goal_x_ = goal_path_.poses[goal_path_.poses.size() - 1].pose.position.x;
                    global_goal_y_ = goal_path_.poses[goal_path_.poses.size() - 1].pose.position.y;
                    std::stringstream ss;
                    ss << goal_path_.header.stamp.sec;
                    ss >> msg_goal_reached.goal_id.id;
                    attempts = (int)(get_path_total_distance(goal_path_)/max_linear_speed_*4*RATE + 5*RATE);
                }
                break;

            
            case SM_GOAL_POSE_ACCEL:
                get_robot_position_wrt_odom();
                global_error = sqrt((goal_x_ - robot_x_)*(goal_x_ - robot_x_) + (goal_y_ - robot_y_)*(goal_y_ - robot_y_));

                if(global_error < fine_dist_tolerance_)
                    state = SM_GOAL_POSE_CORRECT_ANGLE;
                else if(global_error < current_linear_speed*current_linear_speed/(linear_acceleration_*5))
                {
                    state = SM_GOAL_POSE_DECCEL;
                    temp_k = current_linear_speed/sqrt(global_error);
                }
                else if(current_linear_speed >= max_linear_speed_)
                {
                    state = SM_GOAL_POSE_CRUISE;
                    current_linear_speed = max_linear_speed_;
                }
                if(--attempts <= 0)
                {
                    state = SM_GOAL_POSE_FAILED;
                    std::cout << "SimpleMove.-> Timeout exceeded while trying to reach goal position. Current state: GOAL_POSE_ACCEL." << std::endl;
                }
                pub_cmd_vel_->publish(calculate_speeds(robot_x_, robot_y_, robot_t_, goal_x_, goal_y_, 
                                                      min_linear_speed_, current_linear_speed, max_angular_speed_, 
                                                      alpha_*2, beta_/4, goal_distance_ < 0, move_lat_));
                current_linear_speed += (linear_acceleration_*5)/RATE;
                break;
                
            case SM_GOAL_POSE_CRUISE:
                get_robot_position_wrt_odom();
                global_error = sqrt((goal_x_ - robot_x_)*(goal_x_ - robot_x_) + (goal_y_ - robot_y_)*(goal_y_ - robot_y_));
                if(global_error < fine_dist_tolerance_)
                    state = SM_GOAL_POSE_CORRECT_ANGLE;
                else if(global_error < current_linear_speed*current_linear_speed/(linear_acceleration_*5))
                {
                    state = SM_GOAL_POSE_DECCEL;
                    temp_k = current_linear_speed/sqrt(global_error);
                }
                if(--attempts <= 0)
                {
                    state = SM_GOAL_POSE_FAILED;
                    std::cout << "SimpleMove.-> Timeout exceeded while trying to reach goal position. Current state: GOAL_POSE_CRUISE." << std::endl;
                }
                pub_cmd_vel_->publish(calculate_speeds(robot_x_, robot_y_, robot_t_, goal_x_, goal_y_, 
                                                      min_linear_speed_, current_linear_speed, max_angular_speed_,
                                                      alpha_*2, beta_/4, goal_distance_ < 0, move_lat_));
                break;

            case SM_GOAL_POSE_DECCEL:
                get_robot_position_wrt_odom();
                global_error = sqrt((goal_x_ - robot_x_)*(goal_x_ - robot_x_) + (goal_y_ - robot_y_)*(goal_y_ - robot_y_));

                if(global_error < fine_dist_tolerance_)
                        state = SM_GOAL_POSE_CORRECT_ANGLE;
                if(--attempts <= 0)
                {
                    state = SM_GOAL_POSE_FAILED;
                    std::cout << "SimpleMove.-> Timeout exceeded while trying to reach goal position. Current state: GOAL_POSE_DECCEL." << std::endl;
                }
                current_linear_speed = temp_k * sqrt(global_error);
                if(current_linear_speed < min_linear_speed_) current_linear_speed = min_linear_speed_;
                pub_cmd_vel_->publish(calculate_speeds(robot_x_, robot_y_, robot_t_, goal_x_, goal_y_, 
                                                       min_linear_speed_, current_linear_speed, max_angular_speed_,
                                                       alpha_*2, beta_/4, goal_distance_ < 0, move_lat_,
                                                       use_pot_fields_, rejection_force_.y));
                break;

            case SM_GOAL_POSE_CORRECT_ANGLE:
                get_robot_position_wrt_odom();
                global_error = (goal_t_ - robot_t_);
                if (global_error > M_PI) global_error-=2*M_PI;
                if (global_error <= -M_PI) global_error+=2*M_PI;
                global_error = fabs(global_error);
                if(global_error < angle_tolerance_)
                    state = SM_GOAL_POSE_FINISH;
                pub_cmd_vel_->publish(calculate_speeds(robot_t_, goal_t_, max_angular_speed_, beta_/4));
                if(--attempts <= 0)
                {
                    state = SM_GOAL_POSE_FAILED;
                    std::cout << "SimpleMove.-> Timeout exceeded while trying to reach goal position. Current state: GOAL_POSE_CORRECT_ANGLE." << std::endl;
                }   
                break;

            case SM_GOAL_POSE_FINISH:
                std::cout << "SimpleMove.-> Successful move with dist=" << goal_distance_ << " angle=" << goal_angle_ << std::endl;
                state = SM_INIT;
                msg_goal_reached.status = actionlib_msgs::msg::GoalStatus::SUCCEEDED;
                pub_goal_reached_->publish(msg_goal_reached);
                pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
                current_linear_speed = 0;
                break;

            case SM_GOAL_POSE_FAILED:
                std::cout << "SimpleMove.-> FAILED move with dist=" << goal_distance_ << " angle=" << goal_angle_ << std::endl;
                state = SM_INIT;
                msg_goal_reached.status = actionlib_msgs::msg::GoalStatus::ABORTED;
                pub_goal_reached_->publish(msg_goal_reached);
                pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
                current_linear_speed = 0;
                break;

            case SM_GOAL_PATH_ACCEL:
                if(collision_risk_)
                {
                    state = SM_GOAL_PATH_FAILED;
                    pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
                    std::cout << "SimpleMove.-> WARNING! Collision risk detected!!!!!" << std::endl;
                }
                else{
                    get_robot_position_wrt_map();
                    get_next_goal_from_path(prev_pose_idx, next_pose_idx);
                    global_error = sqrt((global_goal_x_ - robot_x_)*(global_goal_x_ - robot_x_) + (global_goal_y_ - robot_y_)*(global_goal_y_- robot_y_));

                    if(global_error < coarse_dist_tolerance_)
                    {
                        state = SM_GOAL_PATH_FINISH;
                    }
                    else if(global_error < current_linear_speed*current_linear_speed/linear_acceleration_)
                    {
                        state = SM_GOAL_PATH_DECCEL;
                        temp_k = current_linear_speed/sqrt(global_error);
                    }
                    else if(current_linear_speed >= max_linear_speed_)
                    {
                        state = SM_GOAL_PATH_CRUISE;
                        current_linear_speed = max_linear_speed_;
                    }
                    if(--attempts <= 0)
                    {
                        state = SM_GOAL_PATH_FAILED;
                        std::cout<<"SimpleMove.-> Timeout exceeded while trying to reach goal path. Current state: GOAL_PATH_ACCEL."<<std::endl;
                    }
                    pub_cmd_vel_->publish(calculate_speeds(robot_x_, robot_y_, robot_t_, goal_x_, goal_y_, 
                                                           min_linear_speed_, current_linear_speed, max_angular_speed_,
                                                           alpha_, beta_, false, move_lat_,
                                                           use_pot_fields_, rejection_force_.y));
                    if(move_head_) pub_head_goal_pose_->publish(get_next_goal_head_angles(next_pose_idx));
                    current_linear_speed += linear_acceleration_/RATE;
                }
                break;

            case SM_GOAL_PATH_CRUISE:
                if(collision_risk_)
                {
                    state = SM_GOAL_PATH_FAILED;
                    pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
                    std::cout << "SimpleMove.-> WARNING! Collision risk detected!!!!!" << std::endl;
                }
                else
                {
                    get_robot_position_wrt_map();
                    get_next_goal_from_path(prev_pose_idx, next_pose_idx);
                    global_error = sqrt((global_goal_x_ - robot_x_)*(global_goal_x_ - robot_x_) + (global_goal_y_ - robot_y_)*(global_goal_y_ - robot_y_));
                    if(global_error < coarse_dist_tolerance_)
                        state = SM_GOAL_PATH_FINISH;
                    else if(global_error < current_linear_speed*current_linear_speed/linear_acceleration_)
                    {
                        state = SM_GOAL_PATH_DECCEL;
                        temp_k = current_linear_speed/sqrt(global_error);
                    }
                    if(--attempts <= 0)
                    {
                        state = SM_GOAL_PATH_FAILED;
                        std::cout<<"SimpleMove.-> Timeout exceeded while trying to reach goal path. Current state: GOAL_PATH_CRUISE."<<std::endl;
                    }
                    pub_cmd_vel_->publish(calculate_speeds(robot_x_, robot_y_, robot_t_, goal_x_, goal_y_, 
                                                           min_linear_speed_, current_linear_speed, max_angular_speed_, 
                                                           alpha_, beta_, false, move_lat_,
                                                           use_pot_fields_, rejection_force_.y));
                    if(move_head_) pub_head_goal_pose_->publish(get_next_goal_head_angles(next_pose_idx));
                }
                break;

            case SM_GOAL_PATH_DECCEL:
                if(collision_risk_)
                {
                    state = SM_GOAL_PATH_FAILED;
                    pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
                    std::cout << "SimpleMove.-> WARNING! Collision risk detected!!!!!" << std::endl;
                }
                else
                {
                    get_robot_position_wrt_map();
                    get_next_goal_from_path(prev_pose_idx, next_pose_idx);
                    global_error = sqrt((global_goal_x_ - robot_x_)*(global_goal_x_ - robot_x_) + (global_goal_y_ - robot_y_)*(global_goal_y_ - robot_y_));
                    if(global_error < coarse_dist_tolerance_)
                        state = SM_GOAL_PATH_FINISH;
                    if(--attempts <= 0)
                    {
                        state = SM_GOAL_PATH_FAILED;
                        std::cout<<"SimpleMove.-> Timeout exceeded while trying to reach goal path. Current state:GOAL_PATH_DECCEL."<<std::endl;
                    }
                    current_linear_speed = temp_k * sqrt(global_error);
                    if(current_linear_speed < min_linear_speed_) current_linear_speed = min_linear_speed_;
                    pub_cmd_vel_->publish(calculate_speeds(robot_x_, robot_y_, robot_t_, goal_x_, goal_y_, 
                                                           min_linear_speed_, current_linear_speed, max_angular_speed_,
                                                           alpha_, beta_, false, move_lat_,
                                                           use_pot_fields_, rejection_force_.y));
                    if(move_head_) pub_head_goal_pose_->publish(get_next_goal_head_angles(next_pose_idx));
                }
                break;

            case SM_GOAL_PATH_FINISH:
                std::cout << "SimpleMove.-> Path succesfully followed." << std::endl;
                state = SM_INIT;
                msg_goal_reached.status = actionlib_msgs::msg::GoalStatus::SUCCEEDED;
                pub_goal_reached_->publish(msg_goal_reached);
                pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
                current_linear_speed = 0;
                break;

            case SM_GOAL_PATH_FAILED:
                std::cout << "SimpleMove.-> FAILED path traking." << std::endl;
                state = SM_INIT;
                msg_goal_reached.status = actionlib_msgs::msg::GoalStatus::ABORTED;
                pub_goal_reached_->publish(msg_goal_reached);
                pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
                current_linear_speed = 0;
                break;

            default:
                std::cout<<"SimpleMove.-> A VERY STUPID PERSON PROGRAMMED THIS SHIT. APOLOGIES FOR THE INCONVINIENCE :'(" << std::endl;
                return;
            }

        } 
        catch (const std::exception& e) 
        {
            RCLCPP_ERROR(this->get_logger(), "SimpleMove.-> Error in SimpleMove processing: %s", e.what());
        }
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleMoveNode>();
    //rclcpp::spin(node);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    
    return 0;
}
