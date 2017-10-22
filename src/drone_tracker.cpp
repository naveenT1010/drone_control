#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <iostream>

using namespace std;
using namespace cv;
//using namespace ros;

Rect2d roi;

void drone_tracker(const sensor_msgs::ImageConstPtr& drone_feed){
    /*
     * Need to do following
     * 1. get image msg to opencv image
     * 2. apply the tracker
     * 3. show the feed.
    */
    Mat feed;
    feed = cv_bridge::toCvCopy(drone_feed,sensor_msgs::image_encodings::BGR8)->image;
    Ptr<Tracker> tracker = Tracker::create("KCF");
    if(roi.width==0 || roi.height==0){
	    selectROI("tracker",feed);
	    tracker->init(feed,roi);
	    ROS_INFO("Tracking Started");
    }
    else{
            tracker->update(feed,roi);
	    rectangle(feed,roi,Scalar(0,0,255),2,1);
	    imshow("Tracked", feed);
	    if(waitKey(100)==27)
		   return; 
    }
    //cout<<"version "<<CV_MAJOR_VERSION<<"."<<CV_MINOR_VERSION<<endl;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
}

int main(int argc, char **argv){
    ros::init(argc,argv,"drone_tracker");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //Publisher and Subscriber for drone camera
    ros::Publisher tracked = nh.advertise<sensor_msgs::Image>("kcf_track", 10);
    image_transport::Subscriber cam_feed = it.subscribe("drone/camera/image_raw", 10, drone_tracker);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

//    pose.pose.position.x = 5;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);
	
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
	
}

