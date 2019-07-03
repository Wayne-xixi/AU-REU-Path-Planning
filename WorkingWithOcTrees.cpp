#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ros/ros.h>
//#include <octomap/octomap_timing.h>
#include <octomap/octomap.h>
//#include <octomap/math/Utils.h>
#include <fstream>
//#include <octomap/OcTreeLUT.h>
#include <iostream>
#include <istream>
#include <list>
//#include <octomap_msgs>
//#include <octomap_msgs/conversion.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTreeIterator.hxx>
#include<octomap/AbstractOccupancyOcTree.h>
//#include <"power_plant.bt">

using namespace std;
using namespace octomap;
//using namespace OcTree;

int main(int argc, char** argv){
ros::init (argc, argv, "OcTree_node");
ros::start();

// OcTreeBase* bt = OcTreeBase::read("power_plant.bt");
/**
ifstream fs;
std::ofstream file("power_plant".c_str(), std::ios_base::in |std::ios_base::binary);
fs.open("power_plant.bt");
if (file.is_open()){
ROS_INFO("File is open");

}
else{
ROS_INFO("File is not open");

}*/
//OcTree* bt = new OcTree("power_plant.bt");
AbstractOcTree* tree = AbstractOcTree::read("mapper.ot");
OcTree* bt = dynamic_cast<OcTree*>(tree);
 
  for(OcTree::leaf_iterator it = bt -> begin_leafs(), end = bt -> end_leafs(); it != end; ++it){

    if(bt->isNodeOccupied(*it)){
ROS_INFO("Node Occupied Nodes");
//ROS_INFO( it.getX());
//cout << "  x = " << it.getX() << std::endl;
//ROS_INFO(*it);
     // cout << "Node: " << *it << "is occupied" << "\n";
    } else {
      //cout << "Node: " << *it << "is not occupied" << "\n";
//ROS_INFO("Following Nodes are NOT Occupied.");
//ROS_INFO(*it);
   }
  }
ros::spin();
ros::shutdown();
return 0;
}
