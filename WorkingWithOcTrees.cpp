#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <octomap/octomap_timing.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>

using namespace std;
using namespace octomap;
using namespace OcTree;

int main(){
  OcTree* tree = dynamic_cast<OcTree*>(read(power_plant.bt));
  for(leaf_iterator it = tree -> begin_leafs(), end = tree -> end_leafs(); it != end; ++it){
    if(tree->isNodeOccupied(*it)){
      cout << "Node: " << it << "is occupied" << "\n";
    } else {
      cout << "Node: " << it << "is not occupied" << "\n";
    }
  }
}
