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

class myComparator
{
public:
    int operator() (const Front& cost1, const Front& cost2)
    {
        return cost1.cost > cost2.cost;
    }
};

float distanceBetweenPoints(Point p1, Point p2){
  return(sqrt(pow((p1.x-p2.x),2)+pow((p1.y-p2.y),2)+pow((p1.z-p2.z),2)));
}

//Create Dijkstra's algorithm
vector<Point> Dijkstras(Point start, Point goal, OcTree* octo, int min_x, int max_x, int min_y, int max_y, int min_z, int max_z) {
  //Initialize to first node which has cost 0, it's only option is start, and it came from no other nodes
  //Front front[?] = {{.00001, start, NULL}};
  priority_queue <Front, vector<Front>, myComparator > front;
  front.push({.00001+distanceBetweenPoints(start, goal), .00001, start, {0,0,0}});
  //Initialize visited array to be the size of all of the nodes Initialized all to zeroes
  int env_x = abs(min_x)+max_x;
  int env_y = abs(min_y)+max_y;
  int env_z = abs(min_z)+max_z;
  double visited[env_x][env_y][env_z] = {0};
  //Create an array that keeps track of what each point came from
  Point came_from[env_x][env_y][env_z];
  //Initialize the possible movements which contain (x,y,z) points and the cost to move to those points
  PointCost movements[6] = {{1,0,0,1},{0,1,0,1},{-1,0,0,1},{0,-1,0,1},{0,0,1,1},{0,0,-1,1}};
  Point pos_of_node;
  //Find the node with the minimum cost (not using a heap)
  while(front.size()>0){
    //Before using a priority queue
    /*
    int min_cost = 1000000000000;
    Front min_node;
    for (int i = 0; a<size(front); i++){
      if (front[i].cost<min_cost){
        min_node = front[i];
      }
    }
    */
    //remove the node with the minimum cost
    Front min_node = front.top();
    front.pop();

    //Find the cost, position, and previous node of the current node and store those values
    float totalCost_of_node = min_node.totalCost;
    float cost_of_node = min_node.cost;
    pos_of_node = min_node.pos;
    Point previous_node = min_node.previous;

    //Check if this node has been visited before
    if (visited[pos_of_node.x][pos_of_node.y][pos_of_node.z] > 0){
      continue;
    }
    //if it was not visited, it now is so add it to the visited array
    visited[pos_of_node.x][pos_of_node.y][pos_of_node.z] = cost_of_node;
    //update the came_from array to show the previous node that is associated with this node
    came_from[pos_of_node.x][pos_of_node.y][pos_of_node.z] = previous_node;
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
      new_x=pos_of_node.x+movements[i].x;
      new_y=pos_of_node.y+movements[i].y;
      new_z=pos_of_node.z+movements[i].z;
      if(new_x<min_x || new_x >= max_x || new_y<min_y || new_y>=max_y || new_z<min_z || new_z>=max_z){
        continue;
      }
      Point new_pos;
      new_pos.x = new_x;
      new_pos.y = new_y;
      new_pos.z = new_z;
      double temp_x = new_pos.x;
      double temp_y = new_pos.y;
      double temp_z = new_pos.z;
      OcTreeNode* node = octo->search(temp_x, temp_y, temp_z, 0);
      int occupied = 0;
      if(node == 0 && visited[new_pos.x][new_pos.y][new_pos.z]==0 ){
        for(OcTree::leaf_bbx_iterator it = octo -> begin_leafs_bbx(Vector3 (new_pos.x-0.5,new_pos.y-0.5, new_pos.z-0.5), Vector3 (new_pos.x+ 0.5,new_pos.y+0.5, new_pos.z+0.5)), end = octo -> end_leafs_bbx(); it != end; ++it){
                occupied = octo ->isNodeOccupied(*it);
 		            if(occupied == 1){
		                break;
		            }
        }
        if(occupied == 0){
          float new_cost = cost_of_node+movements[i].cost;
          front.push({new_cost+distanceBetweenPoints(new_pos, goal), new_cost, new_pos, pos_of_node});
        }
      }
      else if(visited[new_pos.x][new_pos.y][new_pos.z]==0 && octo->isNodeOccupied(node) ==0){
        for(OcTree::leaf_bbx_iterator it = octo -> begin_leafs_bbx(Vector3 (new_pos.x-0.5,new_pos.y-0.5, new_pos.z-0.5), Vector3 (new_pos.x+ 0.5,new_pos.y+0.5, new_pos.z+0.5)), end = octo -> end_leafs_bbx(); it != end; ++it){
                      occupied = octo ->isNodeOccupied(*it);
       		            if(occupied == 1){
      	                 break;
      		            }
        }
      	if(occupied == 0){
          float new_cost = cost_of_node+movements[i].cost;
          front.push({new_cost+distanceBetweenPoints(new_pos, goal), new_cost, new_pos, pos_of_node});
        }
     }
  }
}
  //Initialize optimal_path array which contains (x,y,z) points
  vector<Point> path;
  if(pos_of_node.x == goal.x && pos_of_node.y == goal.y && pos_of_node.z == goal.z){
    while(pos_of_node.x != start.x || pos_of_node.y != start.y || pos_of_node.z != start.z){
      path.push_back(pos_of_node);
      pos_of_node = came_from[pos_of_node.x][pos_of_node.y][pos_of_node.z];
    }
    path.push_back(start);
    reverse(path.begin(), path.end());
  }
  return path;
}

int main(){
  //Find number of possible nodes
  //world_extents = (200, 150, 200);
  int min_x = -6;
  int max_x = 6;
  int min_y = -6;
  int max_y = 6;
  int min_z = 0;
  int max_z = 4;
  Point start = {0,0,1};
  Point goal = {5,2,1};
  AbstractOcTree* tree = AbstractOcTree::read("obstaclecourse1.ot");
  OcTree* bt = dynamic_cast<OcTree*>(tree);
  //in python code they create an array of all zeros that will contain a value of 255 if an obstacle is there
  //world_obstacles = np.zeros(world_extents, dtype=np.uint8)
  //int obstacles_in_env[env_x][env_y][env_z] = {0};
  //If there is an obstacle at a node, set that node to be 255
  //STILL NEED TO CODE WHEN THE TIME COMES

  vector<Point> results;
  results = Dijkstras(start, goal, bt, min_x, max_x, min_y, max_y, min_z, max_z);
  cout << "Size of Results : "<< results.size() << endl;
  for (int i = 0; i < results.size(); i ++)
  {
   cout << "(" << results[i].x << ", ";
   cout << results[i].y << ", ";
   cout << results[i].z << ")" << endl;
  }
}
