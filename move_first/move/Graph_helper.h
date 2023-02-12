#include <iostream>
using namespace std;


class GraphHelper {

public:
	char dir_rotation = 'O';
	int degrees_from_act_to_target_pos = 0;

	void commands_for_rotation_from_direction_to_target_dir() {
		dir_rotation = 'R';
		degrees_from_act_to_target_pos = 95;
		cout << "direction, grad return" << endl;
	}

};
