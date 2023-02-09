
#include <iostream>
#include <iomanip>
#include<stdio.h>
using namespace std;
#include "helper_odometry_class.h"

void print_array(float arr[], int len) {
    /* just printing an array
    */
    for (int a = 0; a < len; a++) {
        //std::cout << std::setprecision(4);
        
        //cout << arr[a] << endl;
        printf("%0.5lf\n", arr[a]);
    }

}







class Body {
    //Body(void) { Odometry odo; };
    Helper_odometry helper_odo;
    float actual_x = 0;
    float actual_y = 0;
    float actual_theta = 0;
    float delta_time = 0;
   

public:
   

    float calculated_odometry[3]{ 1,2,3 };
    void odometry(float speed_L, float speed_R) {
        
        float INTERVAL_ODOMETRY = 0.1;
        unsigned long previous_time = 1;
        unsigned long current_time = 2; // millis();
        delta_time = current_time - previous_time;
        if (current_time - previous_time >= INTERVAL_ODOMETRY) {
            if (speed_L == 0 && speed_R == 0) {
                return;
            }

            float x_y_theta[3];
            helper_odo.calculation_position(speed_L,speed_R,delta_time,
                actual_x,actual_y,actual_theta); 
            for (int x=0; x < 3; x++) {
                x_y_theta[x] = helper_odo.position[x];
            }
            cout << "new position is: " << endl;
            print_array(x_y_theta, 3);








        }




    }


    auto move_one_step(int direction, int distance) {
        return direction;


    }



};








/////////////////////////////////////////////////

int main()
{
   

    Body b;
    b.odometry(20.09, 20.091); // answer 7.153,0,0

   
  

    return 0;
}

