#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <virtual_infrastructure_pkg/vehicle_pose.h>


	double x = 0.0;
	double y = 0.0;
	double x_prev = 0.0;
	double y_prev = 0.0;
	double th_prev = 0.0;
	double th = 0.0;
	double vx = 0.0;
	double vy = 0.0;
	double vth = 0.0;
	double dt;

	ros::Time current_time, last_time;
	ros::Publisher odom_pub;
	ros::Subscriber sub;
	tf::TransformBroadcaster odom_broadcaster;


void positionCallback (const virtual_infrastructure_pkg::vehicle_pose::ConstPtr& vehicle_pose) {
		current_time = ros::Time::now();
		dt = (current_time - last_time).toSec();

		// read msg data
		x = vehicle_pose->x;
		y = vehicle_pose->y;
		th = vehicle_pose->th;

		// calculate velocities
		vx = (x-x_prev)/dt;
		vy = (y-y_prev)/dt;
		vth = (th-th_prev)/dt;

		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "vehicle";

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		//set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.child_frame_id = "vehicle";
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;

		//publish the message
		odom_pub.publish(odom);

		last_time = current_time;

}





int main(int argc, char** argv){
	
	ros::init(argc, argv, "vehicle_odom");
	ros::NodeHandle nh_odom;

	current_time = ros::Time::now();
	last_time = ros::Time::now();
	sub = nh_odom.subscribe("/vehicle_pose",20, &positionCallback);
	odom_pub = nh_odom.advertise<nav_msgs::Odometry>("/vehicle_odom", 20);

	ros::spin();

	return 0;

}
