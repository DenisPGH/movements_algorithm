// ctr + alt more lines editing
#include <iostream>
#include <cmath>
using namespace std;



class Helper_odometry {
    double rpm_to_radians = 0.10471975512;
    double radius_wheel = 3.4;
    double width_of_car = 18.0;

private:
    double calculation_omega(double vel_l, double vel_r, double delta_time) {
        double  omega = (((vel_l - vel_r) / (width_of_car)) * radius_wheel)*delta_time;
        return omega;
    }


public:
    void print_3x1_matrix(const double(&m)[3]) {
        int i;
        for (i = 0; i < 3; i++) {
            cout << m[i] << " ";
        }
        cout << endl;

    }

    double position[3]{ 0,0,0 }; //use this in other classes
    void calculation_position(double vel_l, double vel_r,
        double delta_time, double curr_x, double curr_y, double curr_theta) {
        vel_l *= rpm_to_radians;
        vel_r *= rpm_to_radians;
        //curr_theta = curr_theta * (3.1415 / 180);;
        double omega = calculation_omega(vel_l, vel_r,delta_time) ;

        double R = 0;
        if (vel_l != vel_r) {
             R = width_of_car / 2.0 * ((vel_r + vel_l) / (vel_l - vel_r));
        }
        else { R = 0.0; }
       

        double ICC_x = curr_x - R * sin(curr_theta);
        double ICC_y = curr_y + R * cos(curr_theta);

        double R_matrix[3][3] = { {cos(omega), -sin(omega), 0},
                                 {sin(omega), cos(omega), 0},
                                 {0, 0, 1},
        };

        double A[3] = { curr_x - ICC_x,
                        curr_y - ICC_y,
                            curr_theta };

        double B[3] = { ICC_x, ICC_y, omega };


        //float result[3] = R_matrix @ A + B.T;
        double result[3] = {0,0,0};
        //calculation the matrix
      
        for (int i = 0; i < 3 ; i++) { // All array elements
            result[i] = 0;
            for (int j = 0; j < 3; j++) {
                result[j] += R_matrix[j][i] * A[i]; 
            }
                
         }

        
        position[0] = result[0] + B[0];
        position[1] = result[1] + B[1];
        position[2] = result[2] + B[2];

        
        
    }


};