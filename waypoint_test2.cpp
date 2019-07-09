#include <ros/ros.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <std_msgs/String.h>
#include <cstdlib>
#include <mavros_msgs/Waypoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <list>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
	bool connected = current_state.connected;
	bool armed = current_state.armed;
	ROS_INFO("%s", armed ? "" : "DisArmed");
}

int main (int argc, char **argv)
{
	  ros::init(argc, argv, "srv_waypoint");
	  ros::NodeHandle nh;

          mavros_msgs::SetMode auto_set_mode;
          auto_set_mode.request.custom_mode = "AUTO.MISSION";
    



          ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
          ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
	  ros::ServiceClient wp_clear_client = nh.serviceClient<mavros_msgs::WaypointClear>("/mavros/mission/clear");
          ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
	  ros::ServiceClient wp_client = nh.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");
	  ros::ServiceClient set_home_client = nh.serviceClient<mavros_msgs::CommandHome>("/mavros/cmd/set_home");
/**
	  mavros_msgs::WaypointPush wp_push_srv;
	  mavros_msgs::WaypointClear wp_clear_srv;
	  mavros_msgs::CommandHome set_home_srv;
*/
 mavros_msgs::WaypointPush wp_push_srv; 
 mavros_msgs::Waypoint wp;

// WP 0
    wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    wp.command        = mavros_msgs::CommandCode::NAV_TAKEOFF;
    wp.is_current     = true;
    wp.autocontinue   = true;
    wp.x_lat          = 47.3978206;
    wp.y_long         = 8.543987;
    wp.z_alt          = 20.0;
    wp_push_srv.request.waypoints.push_back(wp);

// WP 1
    wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    wp.command        = mavros_msgs::CommandCode::NAV_LAND;
    wp.is_current     = true;
    wp.autocontinue   = true;
    wp.x_lat          = 47.3978206;
    wp.y_long         = 8.543987;
    wp.z_alt          = 5.0;
    wp_push_srv.request.waypoints.push_back(wp);
/*
    // WP 1
    wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    wp.command        = mavros_msgs::CommandCode::NAV_LOITER_TIME;
    wp.is_current     = false;
    wp.autocontinue   = true;
    wp.x_lat          = 47.3962527;
    wp.y_long         = 8.5467917;
    wp.z_alt          = 20;
     wp.param1	      = 10;
    wp.param3			= 2;
	wp.param4			= 1;
    wp_push_srv.request.waypoints.push_back(wp);
    
    // WP 2
    wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    wp.command        = mavros_msgs::CommandCode::NAV_WAYPOINT;
    wp.is_current     = false;
    wp.autocontinue   = true;
    wp.x_lat          = 49.3977783;
    wp.y_long         = 8.547906;
    wp.z_alt          = 20;
    wp_push_srv.request.waypoints.push_back(wp);

    // WP 3
    wp.frame          = mavros_msgs::Waypoint::FRAME_MISSION;
    wp.command        = mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
    wp.is_current     = false;
    wp.autocontinue   = true;
    wp.x_lat          = 0;
    wp.y_long         = 0;
    wp.z_alt          = 0;
    wp_push_srv.request.waypoints.push_back(wp);**/

    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected to PX4!");
    // ARM
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) &&
        arm_cmd.response.success){
        ROS_INFO("Vehicle armed");
    }

    // Send WPs to Vehicle
    if (wp_client.call(wp_push_srv)) {
        ROS_INFO("Send waypoints ok: %d", wp_push_srv.response.success);
        if (current_state.mode != "AUTO.MISSION") {
            if( set_mode_client.call(auto_set_mode) &&
                auto_set_mode.response.mode_sent){
                ROS_INFO("AUTO.MISSION enabled");
            }
        }
    }
    else
        ROS_ERROR("Send waypoints FAILED.");

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}



	  


