#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

#include <sstream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bot_mover");

	ros::NodeHandle n;
	geometry_msgs::Twist mover;
	ros::Publisher move_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	ros::Rate loop_rate(5);
	mover.linear.x=0;	

	while(ros::ok())
	{
	
	mover.linear.x+=1;
	move_pub.publish(mover);

	ros::spinOnce();

	loop_rate.sleep();

	}

	return 0;
}
