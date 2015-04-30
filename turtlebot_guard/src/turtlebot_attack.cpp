#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

int main(int argc, char** argv){
    ros::init(argc, argv, "turtlebot_attack");

    ros::NodeHandle node;

    ros::Publisher turtlebot_vel =
            node.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 10);

    tf::TransformListener listener;

    ros::Rate rate(10.0);
    int counter = 0; // to record the number of times that /ar_marker_4
    // can't be found

    while (node.ok()){
        tf::StampedTransform transform;

        geometry_msgs::Twist vel_msg;

        try{
            listener.lookupTransform("/ar_marker_4", "/base_link",
                                     ros::Time(0), transform);
            counter == 0; // set counter back to 0 once find /ar_marker_4
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            counter += 1;
            std::cout << counter << std::endl;
            // turtlebot turns right after missing /ar_marker_4 frame
            // for every 10 times
            if (counter == 10){
                vel_msg.angular.z = 1;
                turtlebot_vel.publish(vel_msg);
                counter = 0;
            }
            ros::Duration(1.0).sleep();
            continue;
        }

        vel_msg.angular.z = 0.3 * atan2(transform.getOrigin().y(),
                                        transform.getOrigin().x());
        vel_msg.linear.x = 0.4 * sqrt(pow(transform.getOrigin().x(), 2) +
                                      pow(transform.getOrigin().y(), 2));
        turtlebot_vel.publish(vel_msg);

        rate.sleep();
    }
    return 0;
};
