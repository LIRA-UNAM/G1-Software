#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

// Message types
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"

// TF2 (Transform listener)
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2_eigen/tf2_eigen.hpp"

// Action messages (used less directly in ROS 2; here for compatibility)
#include "action_msgs/msg/goal_status.hpp"

//DEBUG
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

// Standard
#include <iostream>
#include <vector>
#include <exception>
///

class PotentialFieldsNode : public rclcpp::Node
{
public:
    PotentialFieldsNode() : Node("potential_fields_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        //############
        // Declare parameters
        this->declare_parameter<bool>("use_namespace",   false);
        this->declare_parameter<bool>("debug",           false);
        this->declare_parameter<bool>("show_image",      false);
        this->declare_parameter<bool>("use_pot_fields",  false);

        this->declare_parameter<bool>("use_lidar",       true);
        this->declare_parameter<bool>("use_point_cloud", false);

        this->declare_parameter<float>("laser_min_x", 0.30);
        this->declare_parameter<float>("laser_max_x", 0.80);
        this->declare_parameter<float>("laser_min_y", -0.30);
        this->declare_parameter<float>("laser_max_y", 0.30);
        this->declare_parameter<float>("laser_min_z", 0.05);
        this->declare_parameter<float>("laser_max_z", 1.50);

        this->declare_parameter<float>("cloud_min_x", 0.30);
        this->declare_parameter<float>("cloud_max_x", 0.80);
        this->declare_parameter<float>("cloud_min_y", -0.30);
        this->declare_parameter<float>("cloud_max_y", 0.30);
        this->declare_parameter<float>("cloud_min_z", 0.05);
        this->declare_parameter<float>("cloud_max_z", 1.50);

        this->declare_parameter<float>("laser_pot_fields_d0",   1.0);
        this->declare_parameter<float>("cloud_pot_fields_d0",   1.0);
        this->declare_parameter<float>("laser_pot_fields_k_rej",1.0);
        this->declare_parameter<float>("cloud_pot_fields_k_rej",1.0);
        this->declare_parameter<float>("no_sensor_data_timeout", 0.5);

        this->declare_parameter<int>("cloud_points_threshold", 100);
        this->declare_parameter<int>("cloud_downsampling",     9);
        this->declare_parameter<int>("lidar_points_threshold", 10);
        this->declare_parameter<int>("lidar_downsampling",     1);

        this->declare_parameter<std::string>("point_cloud_topic", "/points");
        this->declare_parameter<std::string>("laser_scan_topic",  "/scan");
        this->declare_parameter<std::string>("base_link_name",    "base_link");

        // Load parameter values into variables
        this->get_parameter("use_namespace",   use_namespace_);
        this->get_parameter("debug",           debug_);
        this->get_parameter("show_image",      show_img_);
        this->get_parameter("use_pot_fields",  use_pot_fields_);

        this->get_parameter("use_lidar",       use_lidar_);
        this->get_parameter("use_point_cloud", use_cloud_);

        this->get_parameter("laser_min_x", laser_min_x_);
        this->get_parameter("laser_max_x", laser_max_x_);
        this->get_parameter("laser_min_y", laser_min_y_);
        this->get_parameter("laser_max_y", laser_max_y_);
        this->get_parameter("laser_min_z", laser_min_z_);
        this->get_parameter("laser_max_z", laser_max_z_);

        this->get_parameter("cloud_min_x", cloud_min_x_);
        this->get_parameter("cloud_max_x", cloud_max_x_);
        this->get_parameter("cloud_min_y", cloud_min_y_);
        this->get_parameter("cloud_max_y", cloud_max_y_);
        this->get_parameter("cloud_min_z", cloud_min_z_);
        this->get_parameter("cloud_max_z", cloud_max_z_);

        this->get_parameter("laser_pot_fields_d0",    laser_pot_fields_d0_);
        this->get_parameter("cloud_pot_fields_d0",    cloud_pot_fields_d0_);
        this->get_parameter("laser_pot_fields_k_rej", laser_pot_fields_k_rej_);
        this->get_parameter("cloud_pot_fields_k_rej", cloud_pot_fields_k_rej_);
        this->get_parameter("no_sensor_data_timeout", no_sensor_data_timeout_);

        this->get_parameter("cloud_points_threshold", cloud_threshold_);
        this->get_parameter("cloud_downsampling",     cloud_downsampling_);
        this->get_parameter("lidar_points_threshold", lidar_threshold_);
        this->get_parameter("lidar_downsampling",     lidar_downsampling_);

        this->get_parameter("point_cloud_topic", point_cloud_topic_);
        this->get_parameter("laser_scan_topic",  laser_scan_topic_);
        this->get_parameter("base_link_name",    base_link_name_);

        // Setup parameter change callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&PotentialFieldsNode::on_parameter_change, this, std::placeholders::_1));

        //############
        // Publishers
        pub_collision_risk_ = this->create_publisher<std_msgs::msg::Bool>(
            make_name("/navigation/potential_fields/collision_risk"), 
            rclcpp::QoS(10).transient_local());

        pub_pot_fields_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            make_name("/navigation/potential_fields/pot_field_markers"), 
            rclcpp::QoS(10).transient_local());

        pub_pot_fields_rejection_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            make_name("/navigation/potential_fields/pf_rejection_force"), 
            rclcpp::QoS(10).transient_local());


        //############
        // Subscribers
        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::SensorDataQoS(),
            std::bind(&PotentialFieldsNode::callback_cmd_vel, this, std::placeholders::_1));

        sub_enable_ = this->create_subscription<std_msgs::msg::Bool>(
            make_name("/navigation/potential_fields/enable"), 
            rclcpp::SensorDataQoS(),
            std::bind(&PotentialFieldsNode::callback_enable, this, std::placeholders::_1));

        sub_enable_cloud_ = this->create_subscription<std_msgs::msg::Bool>(
            make_name("/navigation/potential_fields/enable_cloud"), 
            rclcpp::SensorDataQoS(),
            std::bind(&PotentialFieldsNode::callback_enable_cloud, this, std::placeholders::_1));

        sub_goal_path_ = this->create_subscription<nav_msgs::msg::Path>(
            make_name("/simple_move/goal_path"), rclcpp::SensorDataQoS(),
            std::bind(&PotentialFieldsNode::callback_goal_path, this, std::placeholders::_1));
        
        //############
        // Wait for transforms
        wait_for_transforms("map", base_link_name_);

        // Simple Move main processing
        processing_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&PotentialFieldsNode::potential_fields_processing, this));

        RCLCPP_INFO(this->get_logger(), "PotentialFields.-> PotentialFieldsNode is ready.");
    }

private:
    //############
    // State variables
    // Node state flags
    bool use_namespace_        = false;
    bool enable_               = false;
    bool collision_risk_lidar_ = false;
    bool collision_risk_cloud_ = false;

    // Motion state
    float current_speed_linear_ = 0.0;
    float current_speed_angular_ = 0.0;

    // Timeout counters
    int no_data_cloud_counter_ = 0;
    int no_data_lidar_counter_ = 0;

    // Rejection force outputs
    geometry_msgs::msg::Vector3 rejection_force_lidar_;
    geometry_msgs::msg::Vector3 rejection_force_cloud_;

    // Goal
    float global_goal_x_ = 999999.0;
    float global_goal_y_ = 999999.0;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Internal parameter values
    bool debug_, show_img_, use_pot_fields_, use_lidar_, use_cloud_;

    float laser_min_x_, laser_max_x_, laser_min_y_, laser_max_y_, laser_min_z_, laser_max_z_;
    float cloud_min_x_, cloud_max_x_, cloud_min_y_, cloud_max_y_, cloud_min_z_, cloud_max_z_;
    float laser_pot_fields_d0_, laser_pot_fields_k_rej_;
    float cloud_pot_fields_d0_, cloud_pot_fields_k_rej_;
    float no_sensor_data_timeout_;
    int cloud_threshold_, cloud_downsampling_;
    int lidar_threshold_, lidar_downsampling_;

    std::string point_cloud_topic_;
    std::string laser_scan_topic_;
    std::string base_link_name_;

    //############
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                  pub_collision_risk_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_pot_fields_markers_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr          pub_pot_fields_rejection_;


    //############
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr       sub_enable_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr       sub_enable_cloud_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr       sub_goal_path_;
    
    // Dynamic subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_;

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
            const std::string &name = param.get_name();

            if (name == "use_namespace")        use_namespace_  = param.as_bool();
            else if (name == "debug")           debug_          = param.as_bool();
            else if (name == "show_image")      show_img_       = param.as_bool();
            else if (name == "use_pot_fields")  use_pot_fields_ = param.as_bool();

            else if (name == "use_lidar")       use_lidar_      = param.as_bool();
            else if (name == "use_point_cloud") use_cloud_      = param.as_bool();

            else if (name == "laser_min_x") laser_min_x_ = param.as_double();
            else if (name == "laser_max_x") laser_max_x_ = param.as_double();
            else if (name == "laser_min_y") laser_min_y_ = param.as_double();
            else if (name == "laser_max_y") laser_max_y_ = param.as_double();
            else if (name == "laser_min_z") laser_min_z_ = param.as_double();
            else if (name == "laser_max_z") laser_max_z_ = param.as_double();

            else if (name == "cloud_min_x") cloud_min_x_ = param.as_double();
            else if (name == "cloud_max_x") cloud_max_x_ = param.as_double();
            else if (name == "cloud_min_y") cloud_min_y_ = param.as_double();
            else if (name == "cloud_max_y") cloud_max_y_ = param.as_double();
            else if (name == "cloud_min_z") cloud_min_z_ = param.as_double();
            else if (name == "cloud_max_z") cloud_max_z_ = param.as_double();

            else if (name == "laser_pot_fields_d0")     laser_pot_fields_d0_    = param.as_double();
            else if (name == "cloud_pot_fields_d0")     cloud_pot_fields_d0_    = param.as_double();
            else if (name == "laser_pot_fields_k_rej")  laser_pot_fields_k_rej_ = param.as_double();
            else if (name == "cloud_pot_fields_k_rej")  cloud_pot_fields_k_rej_ = param.as_double();
            else if (name == "no_sensor_data_timeout")  no_sensor_data_timeout_ = param.as_double();

            else if (name == "cloud_points_threshold")  cloud_threshold_    = param.as_int();
            else if (name == "cloud_downsampling")      cloud_downsampling_ = param.as_int();
            else if (name == "lidar_points_threshold")  lidar_threshold_    = param.as_int();
            else if (name == "lidar_downsampling")      lidar_downsampling_ = param.as_int();

            else if (name == "point_cloud_topic") point_cloud_topic_ = param.as_string();
            else if (name == "laser_scan_topic")  laser_scan_topic_  = param.as_string();
            else if (name == "base_link_name")    base_link_name_    = param.as_string();

            else {
                result.successful = false;
                result.reason = "Unsupported parameter: " + name;
                RCLCPP_WARN(this->get_logger(), "PotentialFields.-> Attempted to update unsupported parameter: %s", name.c_str());
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
                    "PotentialFields.-> Waiting for transform from '%s' to '%s'...", source_frame.c_str(), target_frame.c_str());

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
                                    "PotentialFields.-> Still waiting for transform: %s", ex.what());
            }

            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }

        if (!transform_ok) {
            RCLCPP_WARN(this->get_logger(),
                        "PotentialFields.-> Timeout while waiting for transform from '%s' to '%s'.",
                        source_frame.c_str(), target_frame.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "PotentialFields.-> Transform from '%s' to '%s' is now available.",
                        source_frame.c_str(), target_frame.c_str());
        }
    }

    //############
    // Callback functions
    void callback_point_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        try 
        {
            no_data_cloud_counter_ = 0;
            collision_risk_cloud_ = check_collision_risk_with_cloud(msg, rejection_force_cloud_.x, rejection_force_cloud_.y);
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "PotentialFields.-> Error processing callback_point_cloud: %s", e.what());
        }
    }

    void callback_lidar(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        try 
        {
            no_data_lidar_counter_ = 0;
            collision_risk_lidar_ = check_collision_risk_with_lidar(msg, rejection_force_lidar_.x, rejection_force_lidar_.y);
            
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "PotentialFields.-> Error processing callback_lidar: %s", e.what());
        }
    }

    void callback_enable(const std_msgs::msg::Bool::SharedPtr msg)
    {
        try 
        {
            enable_ = msg->data;

            if (enable_) {
                std::stringstream ss;
                ss << "PotentialFields.->Starting detection using: ";
                if (use_lidar_) ss << "lidar ";
                if (use_cloud_) ss << "point_cloud";
                RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

                if (use_cloud_ && !sub_cloud_) {
                    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                        point_cloud_topic_, rclcpp::SensorDataQoS(),
                        std::bind(&PotentialFieldsNode::callback_point_cloud, this, std::placeholders::_1));
                }

                if (use_lidar_ && !sub_lidar_) {
                    sub_lidar_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                        laser_scan_topic_, rclcpp::SensorDataQoS(),
                        std::bind(&PotentialFieldsNode::callback_lidar, this, std::placeholders::_1));
                }
            }
            else {
                RCLCPP_INFO(this->get_logger(), "PotentialFields.->Stopping obstacle detection...");

                if (sub_cloud_) {
                    sub_cloud_.reset();
                    collision_risk_cloud_ = false;
                }

                if (sub_lidar_) {
                    sub_lidar_.reset();
                    collision_risk_lidar_ = false;
                }
            }
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "PotentialFields.-> Error processing callback_enable: %s", e.what());
        }
    }

    void callback_enable_cloud(const std_msgs::msg::Bool::SharedPtr msg)
    {
        try 
        {
            bool enable = msg->data;
            if (enable && use_cloud_) {
                std::stringstream ss;
                ss << "PotentialFields.->Starting detection using point_cloud";
                RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

                if (!sub_cloud_){
                    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                        point_cloud_topic_, rclcpp::SensorDataQoS(),
                        std::bind(&PotentialFieldsNode::callback_point_cloud, this, std::placeholders::_1));
                    }
            }
            else if (use_cloud_) {
                RCLCPP_INFO(this->get_logger(), "PotentialFields.->Stopping obstacle detection with point cloud...");

                if (sub_cloud_) {
                    sub_cloud_.reset();
                    collision_risk_cloud_ = false;
                }
            }
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "PotentialFields.-> Error processing callback_enable: %s", e.what());
        }
    }

    void callback_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        try 
        {
            current_speed_linear_  = msg->linear.x;
            current_speed_angular_ = msg->angular.z;
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "PotentialFields.-> Error processing callback_cmd_vel: %s", e.what());
        }
    }

    void callback_goal_path(const nav_msgs::msg::Path::SharedPtr msg)
    {
        try 
        {
            if (!msg->poses.empty()) {
                global_goal_x_ = msg->poses.back().pose.position.x;
                global_goal_y_ = msg->poses.back().pose.position.y;
            }  
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "PotentialFields.-> Error processing callback_goal_path: %s", e.what());
        }
    }


    //############
    //Potential Fields additional functions
    bool check_collision_risk_with_lidar(
        const sensor_msgs::msg::LaserScan::SharedPtr msg,
        double &rejection_force_x,
        double &rejection_force_y)
    {
        if (current_speed_linear_ <= 0.0 && !debug_) {
            return false;
        }

        float optimal_x = get_search_distance(laser_max_x_); 
        int obstacle_count = 0;
        int force_count = 0;
        rejection_force_x = 0.0;
        rejection_force_y = 0.0;

        // Get transform to base_link
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_.lookupTransform(
                base_link_name_,  // target frame
                msg->header.frame_id,  // source frame
                tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "PotentialFields.-> TF lookup failed: %s", ex.what());
            return false;
        }
        Eigen::Affine3d tf = tf2::transformToEigen(transform_stamped.transform);

        for (size_t i = 0; i < msg->ranges.size(); i += lidar_downsampling_) {
            float angle = msg->angle_min + i * msg->angle_increment;
            float range = msg->ranges[i];
            if (!std::isfinite(range)) continue;

            Eigen::Vector3d v(range * cos(angle), range * sin(angle), 0);
            v = tf * v;

            if (v.x() > laser_min_x_ && v.x() < optimal_x &&
                v.y() > laser_min_y_ && v.y() < laser_max_y_ &&
                v.z() > laser_min_z_ && v.z() < laser_max_z_)
            {
                obstacle_count++;
            }

            double distance = v.norm();
            if (distance < laser_pot_fields_d0_) {
                double force_mag = laser_pot_fields_k_rej_ * std::sqrt(1.0 / distance - 1.0 / laser_pot_fields_d0_);
                rejection_force_x -= force_mag * v.x() / distance;
                rejection_force_y -= force_mag * v.y() / distance;
                force_count++;
            }
        }

        if (force_count > 0) {
            rejection_force_x /= force_count;
            rejection_force_y /= force_count;
        } else {
            rejection_force_x = 0.0;
            rejection_force_y = 0.0;
        }

        if (debug_) {
            if (obstacle_count > lidar_threshold_ && rejection_force_y != 0.0) {
                RCLCPP_INFO(this->get_logger(), "PotentialFields.-> Laser->Rejection force: x=%.3f y=%.3f",
                            rejection_force_x, rejection_force_y);
            }
        }

        return obstacle_count > lidar_threshold_;
    }

    bool check_collision_risk_with_cloud(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg,
        double& rejection_force_x,
        double& rejection_force_y)
    {
        if (current_speed_linear_ <= 0.0 && !(debug_||show_img_)) {
            return false;
        }

        float optimal_x = get_search_distance(cloud_max_x_);
        int obstacle_count = 0;
        int force_count = 0;
        rejection_force_x = 0;
        rejection_force_y = 0;

        // Setup OpenCV mask
        int w = msg->width;
        int h = msg->height;
        cv::Mat mask = cv::Mat::zeros(h, w, CV_8UC1);

        // Get transform to base_link
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_.lookupTransform(
                base_link_name_,  // target frame
                msg->header.frame_id,  // source frame
                tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "PotentialFields.-> TF lookup failed: %s", ex.what());
            return false;
        }
        Eigen::Affine3d tf = tf2::transformToEigen(transform_stamped.transform);

        // Iterate through point cloud data
        const unsigned char* p = msg->data.data();
        for (size_t i = 0; i < msg->width * msg->height;
            i += cloud_downsampling_, p += cloud_downsampling_ * msg->point_step)
        {
            float x = *reinterpret_cast<const float*>(p);
            float y = *reinterpret_cast<const float*>(p + 4);
            float z = *reinterpret_cast<const float*>(p + 8);

            Eigen::Vector3d v(x, y, z);
            v = tf * v;

            if (v.x() > cloud_min_x_ && v.x() < optimal_x &&
                v.y() > cloud_min_y_ && v.y() < cloud_max_y_ &&
                v.z() > cloud_min_z_ && v.z() < cloud_max_z_)
            {
                obstacle_count++;
            }

            if (//v.norm() < cloud_pot_fields_d0_ &&
                v.x() > cloud_min_x_ && v.x() < cloud_max_x_ &&
                v.y() > cloud_min_y_ && v.y() < cloud_max_y_ &&
                v.z() > cloud_min_z_ && v.z() < cloud_max_z_)
            {
                mask.data[i] = 255;
                float force_mag = cloud_pot_fields_k_rej_ * std::sqrt(1.0 / v.norm() - 1.0 / cloud_pot_fields_d0_);

                if (!std::isnan(force_mag)) {
                    rejection_force_x -= force_mag * v.x() / v.norm();
                    rejection_force_y -= force_mag * v.y() / v.norm();
                    force_count++;
                }
            }
        }

        rejection_force_x = (force_count > 0) ? rejection_force_x / force_count : 0.0;
        rejection_force_y = (force_count > 0) ? rejection_force_y / force_count : 0.0;

        if(show_img_){
            cv::imshow("POTENTIAL FIELDS OBSTACLE", mask);
            cv::waitKey(30);
        }

        if (debug_)
        {
            if (rejection_force_y != 0)
            {
                std::cout << "PotentialFields.cloud->rejection_force_x: "
                        << rejection_force_x << "  rejection_force_y: "
                        << rejection_force_y << std::endl;
            }
        }

        return obstacle_count > cloud_threshold_;
    }

    float get_search_distance(float max_x)
    {
        float robot_x, robot_y, robot_a;
        get_robot_position_wrt_map(robot_x, robot_y, robot_a);
        float dist_to_goal = sqrt(pow(global_goal_x_ - robot_x, 2) + pow(global_goal_x_ - robot_x, 2));
        if(dist_to_goal < max_x)
            return dist_to_goal;
        return max_x;
    }

    void get_robot_position_wrt_map(float &robot_x, float &robot_y, float &robot_t)
    {
        try
        {
            geometry_msgs::msg::TransformStamped transformStamped =
                tf_buffer_.lookupTransform("map", base_link_name_, tf2::TimePointZero);

            robot_x = transformStamped.transform.translation.x;
            robot_y = transformStamped.transform.translation.y;

            tf2::Quaternion q(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);

            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            robot_t = static_cast<float>(yaw);

            return;
        }
        catch (const tf2::TransformException &ex)
        {
            //RCLCPP_WARN(this->get_logger(), "PotentialFields.-> TF Exception: %s", ex.what());
            robot_x = robot_y = robot_t = 0.0f;
            return;
        }
    }

    visualization_msgs::msg::MarkerArray get_force_arrow_markers(
        const geometry_msgs::msg::Vector3& f1,
        const geometry_msgs::msg::Vector3& f2)
    {
        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = base_link_name_;
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "pot_fields";
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.07;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        geometry_msgs::msg::Point origin;
        origin.x = 0.0;
        origin.y = 0.0;
        origin.z = 0.0;

        // Force 1 marker (blue)
        marker.id = 0;
        marker.points.clear();
        marker.points.push_back(origin);
        geometry_msgs::msg::Point p1;
        p1.x = (10.0 / laser_pot_fields_k_rej_) * f1.x;
        p1.y = (10.0 / laser_pot_fields_k_rej_) * f1.y;
        p1.z = 0;
        marker.points.push_back(p1);
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        markers.markers.push_back(marker);

        // Force 2 marker (green)
        marker.id = 1;
        marker.points.clear();
        marker.points.push_back(origin);
        geometry_msgs::msg::Point p2;
        p2.x = (10.0 / laser_pot_fields_k_rej_) * f2.x;
        p2.y = (10.0 / laser_pot_fields_k_rej_) * f2.y;
        p2.z = 0;
        marker.points.push_back(p2);
        marker.color.r = 0.0;
        marker.color.g = 0.5;
        marker.color.b = 0.0;
        markers.markers.push_back(marker);

        // Total force marker (red)
        marker.id = 2;
        marker.points.clear();
        marker.points.push_back(origin);
        geometry_msgs::msg::Point p3;
        p3.x = (10.0 / laser_pot_fields_k_rej_) * (f1.x + f2.x);
        p3.y = (10.0 / laser_pot_fields_k_rej_) * (f1.y + f2.y);
        p3.z = 0;
        marker.points.push_back(p3);
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        markers.markers.push_back(marker);

        return markers;
    }

    //############
    //Potential Fields main processing
    void potential_fields_processing() 
    {
        try
        {
            if (!enable_) {
                return;
            }

            if (use_pot_fields_) {
                geometry_msgs::msg::Vector3 msg_rejection_force;
                msg_rejection_force.x = rejection_force_lidar_.x + rejection_force_cloud_.x;
                msg_rejection_force.y = rejection_force_lidar_.y + rejection_force_cloud_.y;

                if (rejection_force_lidar_.y > 0 && rejection_force_cloud_.y > 0) {
                    msg_rejection_force.x = (rejection_force_lidar_.x + rejection_force_cloud_.x) / 2.0;
                    msg_rejection_force.y = (rejection_force_lidar_.y + rejection_force_cloud_.y) / 2.0;
                }

                pub_pot_fields_rejection_->publish(msg_rejection_force);
                pub_pot_fields_markers_->publish(get_force_arrow_markers(rejection_force_lidar_, rejection_force_cloud_));
            }

            // Publish collision risk status
            std_msgs::msg::Bool msg_collision_risk;
            msg_collision_risk.data = collision_risk_lidar_ || collision_risk_cloud_;
            pub_collision_risk_->publish(msg_collision_risk);

            collision_risk_lidar_ = false;
            collision_risk_cloud_ = false;
        } 
        catch (const std::exception& e) 
        {
            RCLCPP_ERROR(this->get_logger(), "PotentialFields.-> Error in PotentialFields processing: %s", e.what());
        }
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PotentialFieldsNode>();
    //rclcpp::spin(node);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    
    return 0;
}
