#include <sstream>
#include <string>
#include <cmath>
#include <vector>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

class PurePursuit : public rclcpp::Node
{

public:
    PurePursuit() : Node("pure_pursuit_node")
    {
        // odom sub

        // waypoints viz pub
        waypoints_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoints", 10);
        loadWaypoints(x_points, y_points);

        // drive pub
        drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    }

    ~PurePursuit() {}

private:
    // variables
    vector<float> x_points;
    vector<float> y_points;

    /// ROS subscribers and publishers
    // odom
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_pub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;

    void poseCallback(const geometry_msgs::msg::PoseStamped::ConstPtr &pose_msg)
    {
        // TODO: find the current waypoint to track using methods mentioned in lecture

        // TODO: calculate curvature/steering angle

        // TODO: publish drive message, don't forget to limit the steering angle.
    }

    void loadWaypoints(vector<float>& x_points, vector<float>& y_points)
    {    
        ifstream csv_file("/sim_ws/src/f1tenth_lab5_sub/waypoints/waypoints.csv");
        string line;
        auto marker_array = visualization_msgs::msg::MarkerArray();
        size_t id = 0;
        const float fixed_z = 0.0;
        int line_count = 0;
        int skip = 50;

        while (getline(csv_file, line))
        {
            ++line_count;
            // only process every 50th line
            if (line_count % skip != 0) continue;

            stringstream line_stream(line);
            string cell;
            vector<float> waypoint;

            while (getline(line_stream, cell, ','))
            {
                waypoint.push_back(stof(cell));
            }
            RCLCPP_INFO(this->get_logger(), "x: %f", waypoint[0]);
            RCLCPP_INFO(this->get_logger(), "y: %f", waypoint[1]);


            if (waypoint.size() >= 2) // x, y
            {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = this->get_clock()->now();
                marker.ns = "waypoints";
                marker.id = id++;
                marker.type = visualization_msgs::msg::Marker::CYLINDER;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.position.x = waypoint[0];
                marker.pose.position.y = waypoint[1];
                marker.pose.position.z = fixed_z;
                marker.scale.x = 0.1; // Diameter of the cylinder in X
                marker.scale.y = 0.1; // Diameter of the cylinder in Y
                marker.scale.z = 0.1; // Height of the cylinder
                marker.color.a = 1.0; // Don't forget to set the alpha!
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;

                marker_array.markers.push_back(marker);
            }
        }
        waypoints_pub->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), "publishing waypoints? %s", "YES");

    }
    

    // pair<float> getNextWaypoint()
    // {
    //     // get current location
    //     // find closest waypoint (l2 norm)
    //     // iterate forward until waypoint past the L2 norm
    //     // set as new goal point
    // }

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}