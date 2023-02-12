#include <vector>
#include <map>
#include <iostream>
using namespace std;

class EdgeRotation {
    int source;
    int destination;
    int weight;
    char direction;
    EdgeRotation(int s, int d, int w, char dir) {
        source = s;
        destination = d;
        weight = w;
        direction = dir;
    }


};


class RotationGraphNodes {
    //dict grapp
    //array [0,..360]
    vector <int> directions;
    map<int,int> child;
    map<int, map<int, int>> graph;
public:

    void fill_directions(){
        for (int i = 0; i <= 360; i++) {
            directions.push_back(i);
        }

    }

    void rot_graph() {
        fill_directions();
        for (int dir : directions) {
            //cout << dir << endl;
            if (!graph.count(dir)){
                cout << dir << endl;
            }
        }
    }
    

    
};
