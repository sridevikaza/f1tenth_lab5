#include <sstream>
#include <string>
#include <cmath>
#include <vector>
#include <fstream>
#include <algorithm>

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

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/point_stamped.h"

using std::placeholders::_1;
using namespace std;

class PurePursuit : public rclcpp::Node
{

public:
    PurePursuit() : Node("pure_pursuit_node"), tf_buffer(std::make_shared<tf2_ros::Buffer>(this->get_clock())), tf_listener(*tf_buffer)
    {
        // odom sub
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 10, std::bind(&PurePursuit::odomCallback, this, _1));

        // drive pub
        drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

        // visualization pubs
        waypoints_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoints", 10);
        goal_pub = this->create_publisher<visualization_msgs::msg::Marker>("/goal", 10);
        loadWaypoints(x_points, y_points);

        // timer to publish markers
        viz_timer = this->create_wall_timer(100ms, std::bind(&PurePursuit::publishMarkers, this));

    }

    ~PurePursuit() {}

private:
    // variables
    float L = 1.0; //lookahead dist
    float steering_gain = 0.3;
    float min_steer = -M_PI/3;
    float max_steer = M_PI/3;
    float velocity = 0.3;
    visualization_msgs::msg::MarkerArray marker_array = visualization_msgs::msg::MarkerArray();
    visualization_msgs::msg::Marker goal_marker = visualization_msgs::msg::Marker();
    vector<float> x_points;
    vector<float> y_points;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    tf2_ros::TransformListener tf_listener;

    /// ROS subscribers and publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_pub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::TimerBase::SharedPtr viz_timer;

    // pure pursuit callback
    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
    {
        // find current waypoint to track
        int goal_idx = 0;
        auto position = odom_msg->pose.pose.position;
        getGoal(position, goal_idx);

        // transform goal point to vehicle frame of reference
        geometry_msgs::msg::PointStamped map_point, base_point;
        map_point.header.frame_id = "map";
        map_point.header.stamp = rclcpp::Time(0);
        map_point.point.x = x_points[goal_idx];
        map_point.point.y = y_points[goal_idx];
        map_point.point.z = 0.0;

        try {
        auto transformStamped = tf_buffer->lookupTransform("ego_racecar/base_link", map_point.header.frame_id,
                                                        map_point.header.stamp, rclcpp::Duration(1, 0));
        tf2::doTransform(map_point, base_point, transformStamped);

        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not transform point: %s", ex.what());
        }

        // calculate curvature/steering angle
        float gamma = 2 * base_point.point.y / pow(L,2);
        float steering_angle = steering_gain * gamma;
        steering_angle = max(min_steer, min(steering_angle, max_steer)); // clip
        RCLCPP_INFO(this->get_logger(), "steering angle: %f", steering_angle);

        // publish drive message
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.speed = velocity;
        drive_msg.drive.steering_angle = steering_angle;
        drive_pub->publish(drive_msg);
    }


    // get the next goal waypoint to move towards
    void getGoal(const geometry_msgs::msg::Point current_point, int& goal_idx)
    {
        // find closest waypoint to current position
        int num_waypoints = x_points.size();
        int closest_idx = 0;
        float closest_dist = numeric_limits<float>::max();
        for (int i=0; i<num_waypoints; ++i)
        {
            float dist = sqrt(pow(x_points[i]-current_point.x, 2) + pow(y_points[i]-current_point.y, 2));
            if (dist < closest_dist)
            {
                closest_idx = i;
                closest_dist = dist;
            }
        }

        // iterate through waypoints until past the lookahead dist to find next goal point
        bool found_goal = false;
        goal_idx = closest_idx;
        while(!found_goal && goal_idx < num_waypoints)
        {
            ++goal_idx;
            float goal_dist = sqrt(pow(x_points[goal_idx]-current_point.x, 2) + pow(y_points[goal_idx]-current_point.y, 2));
            if (goal_dist > L)
            {
                found_goal = true;
            }
        }

        // make goal marker
        goal_marker.header.frame_id = "map";
        goal_marker.header.stamp = this->get_clock()->now();
        goal_marker.ns = "goal";
        goal_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        goal_marker.action = visualization_msgs::msg::Marker::ADD;
        goal_marker.pose.position.x = x_points[goal_idx];
        goal_marker.pose.position.y = y_points[goal_idx];
        goal_marker.pose.position.z = 0.0;
        goal_marker.scale.x = 0.1;
        goal_marker.scale.y = 0.1;
        goal_marker.scale.z = 0.1;
        goal_marker.color.a = 1.0;
        goal_marker.color.r = 1.0;
        goal_marker.color.g = 0.0;
        goal_marker.color.b = 0.0;
    }

    // load pre-recorded waypoints from csv
    void loadWaypoints(vector<float>& x_points, vector<float>& y_points)
    {    
        ifstream csv_file("/sim_ws/src/f1tenth_lab5_sub/waypoints/waypoints2.csv");
        string line;
        size_t id = 0;
        int line_count = 0;
        int skip = 100;

        // read each csv line
        while (getline(csv_file, line))
        {
            ++line_count;
            if (line_count % skip != 0) continue;

            stringstream line_stream(line);
            string cell;
            vector<float> waypoint;

            while (getline(line_stream, cell, ','))
            {
                waypoint.push_back(stof(cell));
            }

            if (waypoint.size() >= 2) // x, y
            {
                // make waypoint marker
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = this->get_clock()->now();
                marker.ns = "waypoints";
                marker.id = id++;
                marker.type = visualization_msgs::msg::Marker::CYLINDER;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.position.x = waypoint[0];
                marker.pose.position.y = waypoint[1];
                marker.pose.position.z = 0.0;
                marker.scale.x = 0.1; 
                marker.scale.y = 0.1; 
                marker.scale.z = 0.1;
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;

                // save marker values
                marker_array.markers.push_back(marker);
                x_points.push_back(waypoint[0]);
                y_points.push_back(waypoint[1]);
            }
        }
    }

    // timer callback to keep publishing markers
    void publishMarkers()
    {
        waypoints_pub->publish(marker_array);
        goal_pub->publish(goal_marker);
    }

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}