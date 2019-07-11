#include <ros/ros.h>
#include <string.h>
#include <gazebo_msgs/GetModelState.h>


using namespace std;

int main(int argc, char **argv){
   ros::init(argc, argv, "Pose");
   ros::NodeHandle nh;
   
   ros::ServiceClient pos_client = nh.serviceClient<gazebo_msgs::GetModelState>
   ("gazebo/get_model_state");
   
   gazebo_msgs::GetModelState pose;
   
   pose.request.model_name = "iris";
   
   while (ros::ok())
   {
   
   pos_client.call(pose);
  
   
   }
   
   
   //cout<< pos << endl;
  


   return 0;
}
