
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include<stdio.h>
#include "helper_odometry_class.h"
#include "kalman_odometry.h"
#include "inverse_matrix.h"
using namespace std;

    
template<typename T>
std::string type_to_string(T data) {
    std::ostringstream o;
    o << data;
    return o.str();
}


void print_position(double (arr)[], int len) {

    /* just printing an array
    */
    for (int a = 0; a < len; a++) {
        cout.precision(5);
        cout << fixed;
        if (a == 2) {
            //double degrees = arr[a] * (3.1415 / 180);
            cout << arr[a] * 57.295779513 << endl;

        }
        else{ cout << arr[a] << endl; }
        
        
        
    }

}







class Body {
    //Body(void) { Odometry odo; };
    Helper_odometry helper_odo;
    KalmanOdometry ekf_b;
    double actual_x = 0;
    double actual_y = 0;
    double actual_theta = 0;
    double delta_time = 0;
   

public:
    double x_y_theta[3];
   

    //double calculated_odometry[3]{ 1,2,3 };
    void odometry(float speed_L, float speed_R) {
        
        double INTERVAL_ODOMETRY = 0.1;
        unsigned long previous_time = 1;
        unsigned long current_time = 2; // millis();
        delta_time = current_time - previous_time;
        if (current_time - previous_time >= INTERVAL_ODOMETRY) {
            if (speed_L != 0 && speed_R != 0) {
                helper_odo.calculation_position(speed_L, speed_R, delta_time,
                    actual_x, actual_y, actual_theta);
                for (int x = 0; x < 3; x++) {
                    x_y_theta[x] = helper_odo.position[x];
                }
                
                cout << "position is: " << endl;
                print_position(x_y_theta, 3);
                // add kalman filter here
                ekf_b.calculation_ekf(x_y_theta,
                    delta_time, speed_L, speed_R);

                for (int x = 0; x < 3; x++) {
                    x_y_theta[x] = ekf_b.state_estimate_k_updated[x];

                }
                cout << "after ekf : " << endl;
                print_position(x_y_theta, 3);
                actual_x = x_y_theta[0];
                actual_y = x_y_theta[1];
                actual_theta = x_y_theta[2];
                
            }

            
         

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
    KalmanOdometry k;
    b.odometry(22.09, 20.091); // answer 7.153 , y: -0.000 , theta: -0.001,ekf= 7.179 , y: -0.020 , theta: 0.007
    b.odometry(22.09, 20.091); // answer 7.153 , y: -0.000 , theta: -0.001,ekf= 7.179 , y: -0.020 , theta: 0.007

    //////// test matrices ///////////////////////////////////

    double matrix1[3][3] = { {2.1, 0.0, 0.0}, 
                            {0.0, 2.1, 0.0},
                            {0.0, 0.0, 2.001} };

    
   /* [[0.476 0.    0. ]
        [0.    0.476 0.]
    [0.    0.    0.5]]*/
    double matrix2[3][3] = { {1, 2, 3}, {4, 5, 6}, {7, 8, 9} };
    double results[3][3] = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };

   

    
    //double* r = &results;
    //k.multiply_3_x_3_matrix(matrix1, matrix2, results);
    //k.transpose_matrix_3_x_3(matrix1, results);
    //k.sum_two_matrices_3_x_3(matrix1, matrix2, results);
    //k.pinv_matrix_3x3(matrix1, results);
    


    /*cout << endl << "Output Matrix: " << endl;
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            cout << results[i][j] << "  ";
        }
        cout << endl;
    }

    cout << endl << "Origin Matrix: " << endl;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            cout << matrix1[i][j] << " ";
        }
        cout << endl;
    }*/



   

   

    


    


   
  

    return 0;
}

