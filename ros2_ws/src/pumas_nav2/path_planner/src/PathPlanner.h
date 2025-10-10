#include <queue>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class PathPlanner
{
public:
    PathPlanner();
    ~PathPlanner();

    static bool AStar(const nav_msgs::msg::OccupancyGrid &map,
                      const nav_msgs::msg::OccupancyGrid &cost_map,
                      const geometry_msgs::msg::Pose &start_pose,
                      const geometry_msgs::msg::Pose &goal_pose,
                      bool diagonal_paths,
                      nav_msgs::msg::Path &result_path);

    static nav_msgs::msg::Path SmoothPath(
                      const nav_msgs::msg::Path& path, 
                      float weight_data = 0.1, 
                      float weight_smooth = 0.9, 
                      float tolerance = 0.00001);
};

class Node
{
public:
    Node();
    ~Node();

    int   index;           //The index of the corresponding cell in the occupancy grid.
    float g_value;        //The accumulated distance of this node.
    float f_value;         //The f-value, used only in the A* algorithm.
    bool  in_open_list;    //A value indicating whether this node is in the open list or not.
    bool  in_closed_list;  //A value indicating whether this node is in the closed list or not.
    Node* parent;          //A pointer to the parent of this node.
};

class CompareByDistance
{
public:
    bool operator()(Node* n1, Node* n2) { return n1->g_value > n2->g_value; }
};

class CompareByFValue
{
public:
    bool operator()(Node* n1, Node* n2) { return n1->f_value > n2->f_value; }
};
