#include<iostream>
#include<algorithm>
#include<utility>
#include <vector>
#include <queue>
#include <map>
#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ros/ros.h>
#include <octomap/octomap.h>
#include <fstream>
#include <istream>
#include <list>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTreeIterator.hxx>
#include<octomap/AbstractOccupancyOcTree.h>
#include <time.h>

using namespace std;
using namespace octomap;
using namespace octomath;

//3D point structure
struct Point
{
  int x, y, z;
};

//Point structure including cost
struct PointCost
{
  int x, y, z;
  float cost;
};

//Keeps track of future movements from current node
struct Front
{
  //Cost between start and Goal + node cost
  float totalCost;
  //Node cost
  float cost;
  Point pos;
  Point previous;
};

//Comparator to compare Front structs by cost
class myComparator
{
public:
    int operator() (const Front& cost1, const Front& cost2)
    {
        return cost1.cost > cost2.cost;
    }
};

//Calculates the distance between two points
float distanceBetweenPoints(Point p1, Point p2){
  return(sqrt(pow((p1.x-p2.x),2)+pow((p1.y-p2.y),2)+pow((p1.z-p2.z),2)));
}

//Create Dijkstra's algorithm
vector<Point> Dijkstras(Point start, Point goal, OcTree* octo) {
   //Time the function
   clock_t t;
   t = clock();
  //The size of our environment 
  int min_x = -36;
  int max_x = 36;
  int min_y = -36;
  int max_y = 36;
  int min_z = 0;
  int max_z = 72;
  //Create a priority queue to keep track of Front objects
  priority_queue <Front, vector<Front>, myComparator > front;
  //Initialize to first node which has cost 0+the distance between the point and the goal, it's only option is start, and it came from no other nodes
  front.push({.00001+distanceBetweenPoints(start, goal), .00001, start, {0,0,0}});
  //Find dimensions of environment
  int env_x = abs(min_x)+max_x;
  int env_y = abs(min_y)+max_y;
  int env_z = abs(min_z)+max_z;
  //Initialize visited array to be the size of all of the nodes Initialized all to zeroes
  double visited[env_x][env_y][env_z] = {0};
  //Create an array that keeps track of what each point came from
  Point came_from[env_x][env_y][env_z];
  //Initialize the possible movements which contain (x,y,z) points and the cost to move to those points
  PointCost movements[6] = {{1,0,0,1},{0,1,0,1},{-1,0,0,1},{0,-1,0,1},{0,0,1,1},{0,0,-1,1}};
  Point pos_of_node;
  //While we haven't searched every node and we haven't reached the goal
  while(front.size()>0){

    //remove the node with the minimum cost
    Front min_node = front.top();
    front.pop();

    //Find the totalCost, cost, position, and previous node of the current node and store those values
    float totalCost_of_node = min_node.totalCost;
    float cost_of_node = min_node.cost;
    pos_of_node = min_node.pos;
    Point previous_node = min_node.previous;

    //Account for negative numbers when indexing arrays
    int x_array = pos_of_node.x;
    if(pos_of_node.x < 0){
    	x_array = abs(pos_of_node.x)+max_x-1;
    }
    int y_array = pos_of_node.y;
    if(pos_of_node.y < 0){
    	y_array = abs(pos_of_node.y)+max_y-1;
    }
    //If the node was visited, go back to beginning of while loop
    if (visited[x_array][y_array][pos_of_node.z] > 0){
      continue;
    }
    //if it was not visited, it now is so add it to the visited array
    visited[x_array][y_array][pos_of_node.z] = cost_of_node;
    //update the came_from array to show the previous node that is associated with this node
    came_from[x_array][y_array][pos_of_node.z] = previous_node;
    //Are we at the goal?
    if (pos_of_node.x == goal.x && pos_of_node.y == goal.y && pos_of_node.z == goal.z) {
      break;
    }
    //Coordinates of new current node
    int new_x = 0;
    int new_y = 0;
    int new_z = 0;
    //Checking the neighbors
    for(int i = 0; i < 6; i++){
      //Creating a new neighboring node
      new_x=pos_of_node.x+movements[i].x;
      new_y=pos_of_node.y+movements[i].y;
      new_z=pos_of_node.z+movements[i].z;
      //Checking that the node is within our environment
      if(new_x<min_x || new_x >= max_x || new_y<min_y || new_y>=max_y || new_z<min_z || new_z>=max_z){
        continue;
      }
      Point new_pos;
      new_pos.x = new_x;
      new_pos.y = new_y;
      new_pos.z = new_z;
      int new_x_array = new_pos.x;
      if(new_pos.x < 0){
      	new_x_array = abs(new_pos.x)+max_x-1;
      }
      int new_y_array = new_pos.y;
      if(new_pos.y < 0){
    	  new_y_array = abs(new_pos.y)+max_y-1;
      }
      //Convert to double for search function
      double temp_x = new_pos.x;
      double temp_y = new_pos.y;
      double temp_z = new_pos.z;
      //Search for the node in the octomap
      OcTreeNode* node = octo->search(temp_x, temp_y, temp_z, 0);
      int occupied = 0;
      //If the node doesn't exist (node == 0) assume that it is unoccupied
      //Check that the node doesn't exist and is unvisited
      if(node == 0 && visited[new_x_array][new_y_array][new_pos.z]==0 ){
	//Check nodes within .8 meters of node to ensure that there is enough space for the drone to fly
        for(OcTree::leaf_bbx_iterator it = octo -> begin_leafs_bbx(Vector3 (new_pos.x-0.8,new_pos.y-0.8, new_pos.z-0.8), Vector3 (new_pos.x+ 0.8,new_pos.y+0.8, new_pos.z+0.8)), end = octo -> end_leafs_bbx(); it != end; ++it){
                occupied = octo ->isNodeOccupied(*it);
			    //If a node is occupied, break the loop
 		            if(occupied == 1){
		                break;
		            }
        }
        //If no surrounding nodes are occupied, push the node onto the front
        if(occupied == 0){
          float new_cost = cost_of_node+movements[i].cost;
          front.push({new_cost+distanceBetweenPoints(new_pos, goal), new_cost, new_pos, pos_of_node});
        }
      }
      //If the node is unoccupied and unvisited
      else if(visited[x_array][y_array][new_pos.z]==0 && octo->isNodeOccupied(node) ==0){
        //Check nodes within .8 meters of node to ensure that there is enough space for the drone to fly
        for(OcTree::leaf_bbx_iterator it = octo -> begin_leafs_bbx(Vector3 (new_pos.x-0.8,new_pos.y-0.8, new_pos.z-0.8), Vector3 (new_pos.x+ 0.8,new_pos.y+0.8, new_pos.z+0.8)), end = octo -> end_leafs_bbx(); it != end; ++it){
                      occupied = octo ->isNodeOccupied(*it);
                      //If a node is occupied, break the loop
       		      if(occupied == 1){
      	                 break;
      		      }
        }
        //If no surrounding nodes are occupied, push the node onto the front
      	if(occupied == 0){
          float new_cost = cost_of_node+movements[i].cost;
          front.push({new_cost+distanceBetweenPoints(new_pos, goal), new_cost, new_pos, pos_of_node});
        }
     }
  }
}
  //Initialize optimal_path array which contains (x,y,z) points
  vector<Point> path;
  //If we broke out of the loop because we found the goal
  if(pos_of_node.x == goal.x && pos_of_node.y == goal.y && pos_of_node.z == goal.z){
    //while our current position isn't the start position
    while(pos_of_node.x != start.x || pos_of_node.y != start.y || pos_of_node.z != start.z){
      int x_array = pos_of_node.x;
      if(pos_of_node.x < 0){
    	  x_array = abs(pos_of_node.x)+max_x-1;
      }
      int y_array = pos_of_node.y;
      if(pos_of_node.y < 0){
    	  y_array = abs(pos_of_node.y)+max_y-1;
      }
      //Add the current node into the path vector
      path.push_back(pos_of_node);
      //change our current position to be the node that our previous node came from
      pos_of_node = came_from[x_array][y_array][pos_of_node.z];
    }
    //Add the start node to our vector
    path.push_back(start);
    //reverse the vector
    reverse(path.begin(), path.end());
  } else {
       cout << "Your goal is unreachable" << endl;
    }
    //End the timer
    t = clock()-t;
    cout <<"This took : " << (float)t/CLOCKS_PER_SEC << " seconds" << endl;
  return path;
}

int main(){
  //Initialize the start and goal nodes
  Point start = {0,0,1};
  Point goal = {5,0,1};
  //read in the octomap
  AbstractOcTree* tree = AbstractOcTree::read("obstaclecourse2.ot");
  OcTree* bt = dynamic_cast<OcTree*>(tree);
  //Call Dijkstras
  vector<Point> results;
  results = Dijkstras(start, goal, bt);
  cout << "Size of Results : "<< results.size() << endl;
  cout << "The Goal is : ( " << goal.x << ", "<< goal.y <<", "<< goal.z <<")"<< endl;
  //Create an output file which contains all of the waypoints
  ofstream outfile ("Waypoints_AStarOT.txt");
  for (int i = 1; i < results.size(); i ++){
    outfile << "0 " << results[i].x << " " << results[i].y << " " << results[i].z << " " << "0" << endl;
  }
outfile.close();
//Use waypoint script information to actually utilize the Points in our file to move from point to point
}
