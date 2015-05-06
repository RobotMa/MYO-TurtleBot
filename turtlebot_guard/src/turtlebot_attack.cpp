#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>

/* Rviz might have to be open before rosrun this node */

int main(int argc, char** argv){
	ros::init(argc, argv, "turtlebot_attack");

	ros::NodeHandle node;

	ros::Publisher turtlebot_vel =
		node.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 10);
	tf::TransformListener listener;

	ros::Rate rate(100.0);
	int counter = 0; // to record the number of times that /ar_marker_13 can't be found

	// Trigger for autonomous control
	std::string control_mode("manual");

	tf::Quaternion q_Y90;
	tf::Transform rot_Y90;
	q_Y90.setRPY(0, 1.57, 0);
	rot_Y90.setRotation(q_Y90);

	while (node.ok()){

		// std:: cout << auto_control << std::endl;
		ros::param::get("/control_mode", control_mode);
		std::cout << "Control mode: " << control_mode << std::endl;

		// When in auto control mode, listen to relative transformation and 
		// track ar_tag #13
		if ( control_mode.compare("auto") == 0 ){

			// std::cout << "Auto working" << std::endl;
			tf::StampedTransform stamped_transform_marker;
			tf::StampedTransform transform;

			geometry_msgs::Twist vel_msg;

			try{
				// listener.lookupTransform("/ar_marker_13",    "/odom",
				//		ros::Time(0), stamped_transform_marker);
				listener.lookupTransform("/ar_marker_13",   "/base_link",
						ros::Time(0), transform);

				counter = 0; // set counter back to 0 once find /ar_marker_4

			}
			catch (tf::TransformException &ex) {
				ROS_ERROR("%s",ex.what());
				counter += 1;
				std::cout << "Transform  from /ar_marker_13 to /odom hasn't been found for " 
					<< counter << " times." << std::endl;

				// turtlebot turns right after missing /ar_marker_13 frame
				// for every 10 times
				if (counter == 10){
					vel_msg.angular.z = 1;
					for (int i = 0; i < 40; i++ ){
						turtlebot_vel.publish(vel_msg);
					}
					counter = 0;
				}
				ros::Duration(1.0).sleep();
				continue;
			}
			
			
			// Rotate /ar_marker_13 by 90 degrees
			// stamped_transform_marker.operator *=(rot_Y90);

			// Recalculate the ralative transform between /ar_marker_13 and /base_link
			// transform.inverseTimes(stamped_transform_marker);
			double scale = 2;
			double co_vel_ang = 0.02;

			vel_msg.angular.z = co_vel_ang * atan2(transform.getOrigin().y(),
					transform.getOrigin().x());
			vel_msg.linear.x = scale*co_vel_ang *sqrt(pow(transform.getOrigin().x(), 2) +
					pow(transform.getOrigin().y(), 2));

			std::cout << "Angular Velocity is: " << vel_msg.angular.z << "rad/s " << std::endl;
			std::cout << "Linear  Velocity is: " << vel_msg.linear.x  << "m/s"    << std::endl;
			turtlebot_vel.publish(vel_msg);
		}
		rate.sleep();
	}
	return 0;
}
