#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

// Message types
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_srvs/srv/empty.hpp>

// Standard
#include <iostream>
#include <vector>
#include <exception>
///

class AugmentedGridMapNode : public rclcpp::Node
{
public:
    AugmentedGridMapNode() : Node("augment_gridmap_online_node")
    {
        // Declare and get parameters
        this->declare_parameter("use_namespace",    false);
        this->declare_parameter("obstacle_radius",  0.05);
        this->declare_parameter("debug",            false);
        this->declare_parameter("input_map",        "map");

        this->get_parameter("use_namespace",        use_namespace_);
        this->get_parameter("obstacle_radius",      obstacle_radius_);
        this->get_parameter("debug",                debug_);
        this->get_parameter("input_map",            input_map_);

        // Subscribers
        sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            input_map_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
            std::bind(&AugmentedGridMapNode::callback_save_map, this, std::placeholders::_1));

        sub_point_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "point_obstacle", rclcpp::SensorDataQoS(),
            std::bind(&AugmentedGridMapNode::callback_add_point, this, std::placeholders::_1));

        // Publishers
        pub_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            make_name("/grid_map/augmented_map"), 
            rclcpp::QoS(10).transient_local());
        pub_metadata_ = this->create_publisher<nav_msgs::msg::MapMetaData>(
            make_name("/grid_map/augmented_map_metadata"), 
            rclcpp::QoS(10).transient_local());
        pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
            make_name("/grid_map/obstacle_markers"), 
            rclcpp::QoS(10).transient_local());

        // Services
        srv_clear_map_ = this->create_service<std_srvs::srv::Empty>(
            make_name("/clear_map"),
            std::bind(&AugmentedGridMapNode::callback_clear_map, this, std::placeholders::_1, std::placeholders::_2));

        srv_get_augmented_map_ = this->create_service<nav_msgs::srv::GetMap>(
            make_name("/grid_map/get_augmented_map"),
            std::bind(&AugmentedGridMapNode::callback_get_augmented_map, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "AugmentedGridMap.-> Map enhancer node Initialization finished");
    }

private:
    // Parameters
    bool use_namespace_, debug_;
    float obstacle_radius_;
    std::string input_map_;

    // Core map storage
    nav_msgs::msg::OccupancyGrid original_map_;
    nav_msgs::msg::OccupancyGrid enhanced_map_;
    nav_msgs::msg::MapMetaData map_metadata_;
    std::vector<geometry_msgs::msg::Point> obstacles_;

    //############
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
    rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr pub_metadata_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;

    //############
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_point_;

    //############
    // Service servers
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_clear_map_;
    rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr srv_get_augmented_map_;

    //############
    // Parameter callback handle
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

        //############
    // Runtime parameter update callback
    rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "AugmentedGridMap.-> Parameters updated successfully.";

        for (const auto &param : params)
        {
            if (param.get_name()      == "use_namespace")       use_namespace_      = param.as_bool();
            else if (param.get_name() == "obstacle_radius")     obstacle_radius_    = param.as_double();
            else if (param.get_name() == "debug")               debug_              = param.as_bool();
            else if (param.get_name() == "input_map")           input_map_          = param.as_string();

            else
            {
                result.successful = false;
                result.reason = "AugmentedGridMap.-> Unsupported parameter: " + param.get_name();
                RCLCPP_WARN(this->get_logger(), "AugmentedGridMap.-> Attempted to update unsupported parameter: %s", param.get_name().c_str());
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

    // Subscribers callbacks
    void callback_save_map(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
    {
        original_map_ = *map;
        enhanced_map_ = *map;
        enhanced_map_.header.stamp = this->now();
        map_metadata_ = map->info;

        RCLCPP_INFO(this->get_logger(), "AugmentedGridMap.-> Got a map of: [%d,%d] @ %f resolution",
                    enhanced_map_.info.width,
                    enhanced_map_.info.height,
                    enhanced_map_.info.resolution);

        pub_map_->publish(enhanced_map_);
        pub_metadata_->publish(map_metadata_);
    }

    void callback_add_point(const geometry_msgs::msg::PointStamped::SharedPtr point)
    {
        if (original_map_.data.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "AugmentedGridMap.-> No map data received yet.");
            return;
        }

        if (point->header.frame_id != original_map_.header.frame_id)
        {
            RCLCPP_ERROR(this->get_logger(), "AugmentedGridMap.-> Frame mismatch: point=%s, map=%s",
                         point->header.frame_id.c_str(),
                         original_map_.header.frame_id.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "AugmentedGridMap.-> Adding point: [%f, %f]", point->point.x, point->point.y);

        add_obstacle_to_map(point->point);
        pub_map_->publish(enhanced_map_);
    }

    // Services callbacks
    void callback_clear_map(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                            std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        enhanced_map_ = original_map_;
        obstacles_.clear();

        RCLCPP_INFO(this->get_logger(), "AugmentedGridMap.-> Map restored to original");

        pub_map_->publish(enhanced_map_);
        pub_metadata_->publish(map_metadata_);
        make_obstacles_markers();
    }

    void callback_get_augmented_map(const std::shared_ptr<nav_msgs::srv::GetMap::Request>,
                                    std::shared_ptr<nav_msgs::srv::GetMap::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "AugmentedGridMap.-> Providing augmented map");
        pub_map_->publish(enhanced_map_);
        pub_metadata_->publish(map_metadata_);
        response->map = enhanced_map_;
    }

    // Additional functions
    void add_obstacle_to_map(const geometry_msgs::msg::Point& pt)
    {
        float x = pt.x;
        float y = pt.y;
        float x0 = map_metadata_.origin.position.x;
        float y0 = map_metadata_.origin.position.y;
        float res = map_metadata_.resolution;

        int cx = (x - x0) / res;
        int cy = (y - y0) / res;

        if (cx < 0 || cx >= static_cast<int>(map_metadata_.width) ||
            cy < 0 || cy >= static_cast<int>(map_metadata_.height))
        {
            RCLCPP_ERROR(this->get_logger(), "AugmentedGridMap.-> Point falls outside map bounds");
            return;
        }

        obstacles_.push_back(pt);

        int min_x = std::max(0, static_cast<int>(cx - obstacle_radius_ / res));
        int max_x = std::min(static_cast<int>(map_metadata_.width), static_cast<int>(cx + obstacle_radius_ / res));
        int min_y = std::max(0, static_cast<int>(cy - obstacle_radius_ / res));
        int max_y = std::min(static_cast<int>(map_metadata_.height), static_cast<int>(cy + obstacle_radius_ / res));

        if (debug_)
        {
            RCLCPP_INFO(this->get_logger(), "AugmentedGridMap.-> Cell: [%d, %d,], Obstacle bounds : X [%d, %d], Y [%d, %d] ", cx, cy, min_x, max_x, min_y, max_y);
        }

        for (int i = min_x; i < max_x; ++i)
        {
            for (int j = min_y; j < max_y; ++j)
            {
                enhanced_map_.data[i + j * map_metadata_.width] = 100;
            }
        }

        make_obstacles_markers();
    }

    void make_obstacles_markers()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = enhanced_map_.header.frame_id;
        marker.header.stamp = this->now();
        marker.ns = "obstacles";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = obstacle_radius_;
        marker.scale.y = obstacle_radius_;
        marker.scale.z = obstacle_radius_;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.points = obstacles_;

        pub_marker_->publish(marker);
    }
};

// main entry point
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AugmentedGridMapNode>();
    //rclcpp::spin(node);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
