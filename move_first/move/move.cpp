#include <vector>
#include <map>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include<stdio.h>
#include "helper_odometry_class.h"
#include "kalman_odometry.h"
#include "inverse_matrix.h"
#include "kalman_rotation.h"
#include "heper_odometry_rotation.h"
#include "Arduino.h"
#include "PID_rotation.h"
#include "kalman_step.h"

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
    KalmanRotation ekf_r;
    KalmanStep ekf_step;
    OdometryRotation odo_rot;
    PID_Rotation pid_rot;
    ControlMotors cm;



    double actual_x = 0;
    double actual_y = 0;
    double actual_theta = 0;
    double delta_time = 0;

    double odo_current_step_theta = 0; // restart theta for the current measurning
    double odo_current_new_x = 0;
    double odo_current_new_y = 0;

    double rad_to_deg = 57.29578;

    double last_time = 0;
   

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



    bool rotation(int direction,char dir_rotation, int  degrees_from_act_to_target_pos) {
        /// <summary>
        /// 
        /// </summary>
        /// <param name="direction"> 0-360</param>
        /// <param name="dir_rotation"> L or R </param>
        /// <param name="degrees_from_act_to_target_pos"> difference from cur deg to wished deg</param>
        /// <returns></returns>
        vector<int> separated_degrees_if_more_than_90;
        odo_current_step_theta = 0;
        odo_current_new_x = 0;
        odo_current_new_y = 0;
        ekf_step.restart_calculation();
        while (true) {
            if ((actual_theta * rad_to_deg) == direction) {
                return true;
            }
            else {
                //// step 30 degrees [90,3]
                int step = 90;
                int delimo = degrees_from_act_to_target_pos / step;
                if ((degrees_from_act_to_target_pos % step) == 0) {
                    for (int i = 0; i <= delimo; i++) {
                        separated_degrees_if_more_than_90.push_back(step);
                        }
                }
                    
                else {
                    for (int i = 0; i <= delimo; i++) {
                        separated_degrees_if_more_than_90.push_back(step);
                        int rest = degrees_from_act_to_target_pos % step;
                        separated_degrees_if_more_than_90.push_back(rest);
                    }

                }
                //////
                for (auto angle : separated_degrees_if_more_than_90) {
                       
                        odo_current_step_theta = 0;
                        odo_current_new_x = 0;
                        odo_current_new_y = 0;
                        ekf_step.restart_calculation();
                        while (true) {
                            double current_time = 1;
                            double delta_time = (current_time - last_time);
                            if (delta_time>= interval_delta_time) {

                                double error_range = 0.5;
                                double coefficient_slow = 1; //1.31
                                double pid_coefficient_low_angle = 30;
                                odo_rot.calculation_rotation(speedRateL, speedRateL,
                                    interval_delta_time,
                                    odo_current_new_x, odo_current_new_y,
                                    odo_current_step_theta);
                                //double current_angle_deg[3] = odo_rot.position_rotation;
                                //cout << " Before : " << current_angle_deg << endl;
                               
                                ekf_step.calculation_step(odo_rot.position_rotation,
                                        double(delta_time), speedRateL, speedRateL);
                                double current_angle_deg_ekf = 0;
                                ekf_step.state_estimate_k_updated; //new values
                                cout << " After : " << ekf_step.state_estimate_k_updated[2] << endl;

                                if (current_angle_deg_ekf * coefficient_slow >= (angle)) {
                                    actual_theta = direction;
                                    odo_current_step_theta = 0;
                                    odo_current_new_x = 0;
                                    odo_current_new_y = 0;
                                    ekf_step.restart_calculation();
                                    break;

                                }
                                else {
                                    //dir_num={'L':2,'R':3}
                                    map <char, int> dir_num{ {'L',2},{'R', 3} };
                                    dir_num['L'];
                                    cout << dir_num[dir_rotation] << endl;
                                    //send comand via PID to both motors in target direction
                                    pid_rot.output();
                                    double L = pid_rot.left_motor_output;
                                    double R = pid_rot.right_motor_output;
                                    cm.movement(L, R, dir_num['L']);
                                }

                                last_time = 2; //update time
                            }
                            //run stop to motors
                        }

                };

                return true;

            }

        }



    }


    auto move_one_step(int direction, int distance, char dir_rotation, int  degrees_from_act_to_target_pos) {
        /// <summary>
        /// 
        /// </summary>
        /// <param name="direction"> 0-360 deg </param>
        /// <param name="distance"> 0-500cm</param>
        /// <param name="dir_rotation"> L or R</param>
        /// <param name="degrees_from_act_to_target_pos"> diff now wished direction</param>
        /// <returns></returns>
        rotation(direction,dir_rotation,degrees_from_act_to_target_pos);
        return direction;


    }



};








/////////////////////////////////////////////////

int main()
{
   

    Body b;
    KalmanOdometry ko;
    b.odometry(22.09, 20.091); // answer 7.153 , y: -0.000 , theta: -0.001,ekf= 7.179 , y: -0.020 , theta: 0.007
    //b.odometry(22.09, 20.091); // answer 7.153 , y: -0.000 , theta: -0.001,ekf= 7.179 , y: -0.020 , theta: 0.007

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

    b.move_one_step(90, 10,'L', 30);
    /*cout << "Rotation: " << endl;
    KalmanRotation kff;
    double resss= kff.calculation_error_rotation(50);
    cout << resss << endl;*/
   



   

   

    


    


   
  

    return 0;
}

