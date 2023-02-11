// ctr + alt more lines editing
#include <iostream>
#include <cmath>



class Helper_odometry {
    float rpm_to_radians = 0.10471975512;
    float radius_wheel = 3.4;
    float width_of_car = 18;

private:
    float calculation_omega(float vel_l, float vel_r) {
        float  omega = ((vel_l - vel_r) / (width_of_car)) * radius_wheel;
        return omega;
    }


public:

    float position[3]{ 0,0,0 }; //use this in other classes
    void calculation_position(float vel_l, float vel_r,
        float delta_time, float curr_x, float curr_y, float curr_theta) {
        vel_l *= rpm_to_radians;
        vel_r *= rpm_to_radians;
        curr_theta = curr_theta * (3.1415 / 180);;
        float omega = calculation_omega(vel_l, vel_r) * delta_time;
        float R = 0;
        if (vel_l != vel_r) {
             R = width_of_car / 2.0 * ((vel_r + vel_l) / (vel_l - vel_r));
        }
        else { R = 0.0; }
       /* cout << R << endl;*/

        float ICC_x = curr_x - R * sin(curr_theta);
        float ICC_y = curr_y + R * cos(curr_theta);

        float R_matrix[3][3] = { {cos(omega), -sin(omega), 0},
                                 {sin(omega), cos(omega), 0},
                                 {0, 0, 1},
        };

        float A[3] = { curr_x - ICC_x,
                        curr_y - ICC_y,
                            curr_theta };

        float B[3] = { ICC_x, ICC_y, omega };


        //float result[3] = R_matrix @ A + B.T;
        float result[3] = {0,0,0};
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