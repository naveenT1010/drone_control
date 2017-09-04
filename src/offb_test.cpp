#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

// for getting current state of the mavros connections
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv){
    ros::init(argc, argc, "offb_test");
    ros::NodeHandle nh;
    
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    while(ros::ok() && current_state.connected){
	ros::spinOnce();
	rate.sleep();
    }
	
    geometry_msgs::PoseStamped drone_pose;
    drone_pose.pose.position.x=0;
    drone_pose.pose.position.y=0;
    drone_pose.pose.position.z=2;
    drone_pose.pose.orientation.x=0;
    drone_pose.pose.orientation.y=0;
    drone_pose.pose.orientation.z=0;
    drone_pose.pose.orientation.w=0;

    for(int i=100; ros::ok() && i>0 ; --i){
	local_pos_pub.publish(drone_pose);
	ros::spinOnce();
	rate.sleep();
    }

	
}
