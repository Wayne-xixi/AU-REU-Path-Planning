#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ros/ros.h>
#include <octomap/octomap.h>
#include <fstream>
#include <iostream>
#include <istream>
#include <list>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTreeIterator.hxx>
#include<octomap/AbstractOccupancyOcTree.h>

using namespace std;
using namespace octomap;


int main(int argc, char** argv){
ros::init (argc, argv, "OcTree_node");
ros::start();
int num_node = 1;


AbstractOcTree* tree = AbstractOcTree::read("mapper.ot");
OcTree* bt = dynamic_cast<OcTree*>(tree);

 
 for(OcTree::leaf_iterator it = bt -> begin_leafs(), end = bt -> end_leafs(); it != end; ++it){
	

    if(bt->isNodeOccupied(*it)){
	std::cout << "Node " << num_node << " is occupied. \n";
        std::cout << "  x = " << it.getX() << std::endl;
        std::cout << "  y = " << it.getY() << std::endl;
        std::cout << "  z = " << it.getZ() << std::endl;
	num_node ++;
    } else {
	std::cout << "Node " << num_node << " is free. \n";
	std::cout << "  x = " << it.getX() << std::endl;
        std::cout << "  y = " << it.getY() << std::endl;
        std::cout << "  z = " << it.getZ() << std::endl;
	num_node ++;
   }

  }
ros::spin();
ros::shutdown();
return 0;
}
