#include <chrono>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"


#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"

#include "sofar_lab/srv/get_obstacle.hpp"

using std::placeholders::_1;

class WaffleController : public rclcpp::Node
{
public:
    WaffleController() : Node("wafflebot_controller")
    {

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&WaffleController::odom_callback, this, _1));

        // //subscribe to rviz 2d goal pose
        goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&WaffleController::goal_callback, this, _1));

            
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

        // tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // // Define our goal coordinates
        // goal_x = 9.0;
        // goal_y = 9.0;
        goal_recieved_ = false;
        // // Declare parameters with default values
        // this->declare_parameter("max_v", 2.0);
        // this->declare_parameter("dt", 1.0); 

        // client
        client_ = this->create_client<sofar_lab::srv::GetObstacle>("get_obstacle");
        
        // Ask the server for the obstacle position (500ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), 
            std::bind(&WaffleController::ask_for_obstacle, this));
    }

private:
void ask_for_obstacle()
    {
        // If the server isn't turned on yet, just ignore it
        if (!client_->service_is_ready()) {
            return;
        }

        // Create an empty request
        auto request = std::make_shared<sofar_lab::srv::GetObstacle::Request>();

        // Send the call and run this mini-function when the server answers
        client_->async_send_request(request, 
            [this](rclcpp::Client<sofar_lab::srv::GetObstacle>::SharedFuture future) {
                obs_x = future.get()->x;
                obs_y = future.get()->y;
            }
        );
    }
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            // Update our target variables with the click from RViz2
            goal_x = msg->pose.position.x;
            goal_y = msg->pose.position.y;
            
            goal_recieved_ = true;

            // Print a message to the terminal so we know it worked!
            RCLCPP_INFO(this->get_logger(), "New Goal Received: X: %.2f, Y: %.2f", goal_x, goal_y);
        }
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) 
    {        
        
        if(!goal_recieved_){ 
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for Goal...");

            return; 
        }   
        //  We have x_curr, y_curr, theta_curr, theta_velocity and Horizon dt
        double x_curr = msg->pose.pose.position.x;
        double y_curr = msg->pose.pose.position.y;
        
        // theta in quaternions => yaw
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        double theta_curr = yaw;

        double best_distance = 1000000;
        double best_theta_vel = 0.0;

        double dt=1;
        double max_v=0.5;
        // this->get_parameter("dt", dt);
        // this->get_parameter("max_v", max_v);
        // proportional velocity for smoothness
        double distance_curr = std::sqrt(std::pow(goal_x - x_curr, 2) + std::pow(goal_y - y_curr, 2));
        double v = distance_curr * 0.6;
        

        if (v > max_v) { v = max_v; }

        for(double theta_velocity = -2.0; theta_velocity <=2.0; theta_velocity +=0.1){

            double theta_avg = theta_curr + (theta_velocity/2.0)*dt;
            

            double x_future = x_curr + (v * dt) * std::cos(theta_avg);
            double y_future = y_curr + (v * dt) * std::sin(theta_avg);

            double dist_to_obs = std::sqrt(std::pow(obs_x - x_future, 2) + std::pow(obs_y - y_future, 2));
            if (dist_to_obs < 1.0) {
                continue;
            }
            double diff_x = goal_x - x_future;
            double diff_y = goal_y - y_future;
            double distance = std::sqrt(std::pow(diff_x, 2) + std::pow(diff_y, 2));
            if (best_distance > distance){
                best_distance = distance;
                best_theta_vel = theta_velocity;
            }
        }

        
        double target_angle = std::atan2(goal_y - y_curr, goal_x - x_curr);


        double raw_error = target_angle - theta_curr;
        // angle wrapping. to not do a whole circle for a small pi error ( 340 degrees error = - 20 degrees )
        double angle_error = std::atan2(std::sin(raw_error), std::cos(raw_error));
        if (std::abs(angle_error) > 0.5){
            v = 0;
        }
        
        auto cmd_msg = geometry_msgs::msg::TwistStamped();
        cmd_msg.header.stamp = this->get_clock()->now();
        cmd_msg.header.frame_id = "base_link"; 

        // threshold 0.1 close to the distance
        if (distance_curr > 0.1) {
            cmd_msg.twist.linear.x = v;
            cmd_msg.twist.angular.z = best_theta_vel;
        } else {
            cmd_msg.twist.linear.x = 0.0;
            cmd_msg.twist.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Goal Reached!");
            goal_recieved_ = false;
        }

        // publish
        publisher_->publish(cmd_msg);



    }

    // Member variables
    double goal_x;
    double goal_y;
    bool goal_recieved_;
    double obs_x = -100.0;
    double obs_y = -100.0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
    rclcpp::Client<sofar_lab::srv::GetObstacle>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaffleController>());
    rclcpp::shutdown();
    return 0;
}
