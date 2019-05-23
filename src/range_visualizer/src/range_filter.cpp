#include "ros/ros.h"
#include "std_msgs/String.h"
#include <vector>
#include "sensor_msgs/Range.h"
#include <sstream>


std::vector<float> myvector;
int average_count=10;

// 
void chatterCallback(const sensor_msgs::Range  &msg)
{
	myvector.push_back(msg.range);
}

int main(int argc, char **argv)
{
//ros::init(argc, argv, "range_publisher");
	ros::init(argc, argv, "filter_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("us_data_raw",1000,chatterCallback);
//ros::Subscriber sub = n.subscribe("fake_range",1000,chatterCallback);
	ros::Publisher range_publisher= n.advertise<sensor_msgs::Range>("filtered_range",1000);
	ros::Rate loop_rate(10);
	int count =0;

// Make relevant objects
	sensor_msgs::Range sensor;
	std_msgs::Header header;
	header.frame_id="range_test";
	sensor.min_range=0.02;
	sensor.max_range=4;

	sensor.field_of_view=0.785398;

//Assign message
	sensor.header=header;
while(ros::ok())
{
//Running average filter of size 10
	if(myvector.size()>(average_count-1)){
		float running_avg_value=0;
		for (int i=0;i<average_count;i++){
			running_avg_value+=myvector[i];
		}	
		running_avg_value/=average_count;
		sensor.range = running_avg_value;
		range_publisher.publish(sensor);
		ROS_INFO("Average after 10 values: %f",running_avg_value);
		myvector.erase(myvector.begin());
	}
	ros::spinOnce();
	

}

return 0;
}
