#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "PathPlanner.h"  

class PathPlannerNode : public rclcpp::Node
{
public:
    PathPlannerNode() : Node("path_planner_node")
    {
        // Declare and get parameters
        this->declare_parameter("use_namespace",        false);
        this->declare_parameter<float>("smooth_alpha",  0.1f);
        this->declare_parameter<float>("smooth_beta",   0.9f);
        this->declare_parameter<bool>("diagonal_paths", false);

        // Initialize internal variables from declared parameters
        this->get_parameter("use_namespace",    use_namespace_);
        this->get_parameter("smooth_alpha",     smooth_alpha_);
        this->get_parameter("smooth_beta",      smooth_beta_);
        this->get_parameter("diagonal_paths",   diagonal_paths_);

        RCLCPP_INFO(this->get_logger(), "PathPlanner.-> Smooth Alpha: %.2f, Beta: %.2f, Diagonal: %s",
                    smooth_alpha_, smooth_beta_, diagonal_paths_ ? "true" : "false");

        // Setup parameter change callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&PathPlannerNode::on_parameter_change, this, std::placeholders::_1));
        
        // Initialize service clients (non-blocking)
        init_service_clients();

        // Advertise planning services
        srv_plan_static_ = this->create_service<nav_msgs::srv::GetPlan>(
            make_name("/path_planner/plan_path_with_static"),
            std::bind(&PathPlannerNode::callback_a_star_with_static_map, this, std::placeholders::_1, std::placeholders::_2));

        srv_plan_augmented_ = this->create_service<nav_msgs::srv::GetPlan>(
            make_name("/path_planner/plan_path_with_augmented"),
            std::bind(&PathPlannerNode::callback_a_star_with_augmented_map, this, std::placeholders::_1, std::placeholders::_2));

        messages_call_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&PathPlannerNode::messages_caller, this));

        RCLCPP_INFO(this->get_logger(), "PathPlanner.-> PathPlannerNode is ready.");
    }

private:
    // Parameters
    bool use_namespace_;
    float smooth_alpha_;
    float smooth_beta_;
    bool diagonal_paths_;

    // Service clients
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr clt_get_static_map_;
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr clt_get_static_cost_map_;
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr clt_get_augmented_map_;
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr clt_get_augmented_cost_map_;

    std::vector<std::thread> threads_;

    // Service servers
    rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr srv_plan_static_;
    rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr srv_plan_augmented_;

    // Parameter callback handle
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // map services
    nav_msgs::msg::OccupancyGrid map_;
    nav_msgs::msg::OccupancyGrid cost_map_;
    nav_msgs::msg::OccupancyGrid augmented_map_;
    nav_msgs::msg::OccupancyGrid augmented_cost_map_;

    // Timer and readiness flag
    rclcpp::TimerBase::SharedPtr messages_call_timer_;
    rclcpp::TimerBase::SharedPtr service_check_timer_;
    bool services_ready_ = false;

    //############
    // Runtime parameter update callback
    rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "PathPlanner.-> Parameters updated successfully.";

        for (const auto &param : params)
        {
            if (param.get_name()      == "use_namespace")   use_namespace_  = param.as_bool();
            else if (param.get_name() == "smooth_alpha")    smooth_alpha_   = param.as_double();
            else if (param.get_name() == "smooth_beta")     smooth_beta_    = param.as_double();
            else if (param.get_name() == "diagonal_paths")  diagonal_paths_ = param.as_bool();

            else
            {
                result.successful = false;
                result.reason = "PathPlanner.-> Unsupported parameter: " + param.get_name();
                RCLCPP_WARN(this->get_logger(), "PathPlanner.-> Attempted to update unsupported parameter: %s", param.get_name().c_str());
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

    //############
    // Initialize service clients (non-blocking)
    void init_service_clients()
    {
        clt_get_static_map_ = this->create_client<nav_msgs::srv::GetMap>(
            make_name("/map_augmenter/get_static_map"));
        clt_get_static_cost_map_ = this->create_client<nav_msgs::srv::GetMap>(
            make_name("/map_augmenter/get_static_cost_map"));
        clt_get_augmented_map_ = this->create_client<nav_msgs::srv::GetMap>(
            make_name("/map_augmenter/get_augmented_map"));
        clt_get_augmented_cost_map_ = this->create_client<nav_msgs::srv::GetMap>(
            make_name("/map_augmenter/get_augmented_cost_map"));

        service_check_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]()
            {
                if (clt_get_static_map_->wait_for_service(std::chrono::seconds(0)) &&
                    clt_get_static_cost_map_->wait_for_service(std::chrono::seconds(0)) &&
                    clt_get_augmented_map_->wait_for_service(std::chrono::seconds(0)) &&
                    clt_get_augmented_cost_map_->wait_for_service(std::chrono::seconds(0)))
                {
                    RCLCPP_INFO(this->get_logger(), "PathPlanner.-> All map services are now available.");
                    services_ready_ = true;
                    service_check_timer_->cancel();
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "PathPlanner.-> Waiting for map services to become available...");
                }
            });
    }

    //############
    // Callback to update messages
    void update_static_maps()
    {
        threads_.push_back(std::thread(std::bind(&PathPlannerNode::call_update_static_maps, this)));
    }
    
    void call_update_static_maps()
    {
        auto req_map = std::make_shared<nav_msgs::srv::GetMap::Request>();
        auto future_map = clt_get_static_map_->async_send_request(req_map);
        try
        {
            auto result_map = future_map.get();
            map_ = result_map->map;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "PathPlanner.-> Received null static map respons");
        }

        auto req_cost = std::make_shared<nav_msgs::srv::GetMap::Request>();
        auto future_cost = clt_get_static_cost_map_->async_send_request(req_cost);
        try
        {
            auto result_cost = future_cost.get();
            cost_map_ = result_cost->map;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "PathPlanner.-> Received null static cost map respons");
        }

        return;
    }
    
    void update_augmented_maps()
    {
        threads_.push_back(std::thread(std::bind(&PathPlannerNode::call_update_augmented_maps, this)));
    }
    
    void call_update_augmented_maps()
    {
        auto req_map = std::make_shared<nav_msgs::srv::GetMap::Request>();
        auto future_map = clt_get_augmented_map_->async_send_request(req_map);
        try
        {
            auto result_map = future_map.get();
            augmented_map_ = result_map->map;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "PathPlanner.-> Received null augmented map respons");
        }

        auto req_cost = std::make_shared<nav_msgs::srv::GetMap::Request>();
        auto future_cost = clt_get_augmented_cost_map_->async_send_request(req_cost);
        try
        {
            auto result_cost = future_cost.get();
            augmented_cost_map_ = result_cost->map;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "PathPlanner.-> Received null augmented cost map respons");
        }

        return;
    }

    // Callback for services
    void callback_a_star_with_static_map(
        const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
        std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
    {
        if (!services_ready_) {
            RCLCPP_ERROR(this->get_logger(), "PathPlanner.-> Services not ready. Cannot handle augmented map request.");
            response->plan.poses.clear(); 
            return;
        }

       //update_static_maps();

        if (map_.data.empty() || cost_map_.data.empty()) {
            response->plan.poses.clear();
            return;
        }

        nav_msgs::msg::Path path;
        bool success = PathPlanner::AStar(map_, cost_map_,
            request->start.pose, request->goal.pose,
            diagonal_paths_, path);

        if (success) {
            nav_msgs::msg::Path smoothed = PathPlanner::SmoothPath(path, smooth_alpha_, smooth_beta_);
            smoothed.header.stamp = this->get_clock()->now();
            smoothed.header.frame_id = "map";  // or your global frame

            if (!smoothed.poses.empty()){
                response->plan = smoothed;
                RCLCPP_INFO(this->get_logger(), "PathPlanner.-> Path planned successfully with size: %d", response->plan.poses.size()); 
            } else {
                RCLCPP_WARN(this->get_logger(), "PathPlanner.-> Failed to plan path.");
                response->plan.poses.clear();
            }
        }
        else {
            RCLCPP_WARN(this->get_logger(), "PathPlanner.-> Failed to plan path.");
            response->plan.poses.clear();
        }
    }


    void callback_a_star_with_augmented_map(
        const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
        std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
    {
        if (!services_ready_) {
            RCLCPP_ERROR(this->get_logger(), "PathPlanner.-> Services not ready. Cannot handle augmented map request.");
            response->plan.poses.clear(); 
            return;
        }

        //update_augmented_maps();

        if (augmented_map_.data.empty() || augmented_cost_map_.data.empty()) {
            response->plan.poses.clear();
            return;
        }

        nav_msgs::msg::Path path;
        bool success = PathPlanner::AStar(augmented_map_, augmented_cost_map_,
            request->start.pose, request->goal.pose,
            diagonal_paths_, path);

        if (success) {
            nav_msgs::msg::Path smoothed = PathPlanner::SmoothPath(path, smooth_alpha_, smooth_beta_);
            smoothed.header.stamp = this->get_clock()->now();
            smoothed.header.frame_id = "map";  // or your global frame

            if (!smoothed.poses.empty()){
                response->plan = smoothed;
                RCLCPP_INFO(this->get_logger(), "PathPlanner.-> Path planned successfully with size: %d", response->plan.poses.size()); 
            } else {
                RCLCPP_WARN(this->get_logger(), "PathPlanner.-> Failed to plan path.");
                response->plan.poses.clear();
            }
        }
        else {
            RCLCPP_WARN(this->get_logger(), "PathPlanner.-> Failed to plan path.");
            response->plan.poses.clear();
        }
    }

    void messages_caller() 
    {
        try
        {
            update_static_maps();
            update_augmented_maps();

        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "PathPlanner.-> Messages call failed.");
        }
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlannerNode>();
    //rclcpp::spin(node);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
