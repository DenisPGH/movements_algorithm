#include <vector>
#include <map>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include<stdio.h>
//#include "kalman_odometry.h"


using namespace std;
double ROBOT_X_STEP = 0.0; //main vaiable
double ROBOT_Y_STEP = 0.0;//main vaiable
double ROBOT_THETA_STEP = 0.0; //radians main vaiable 
const int N = 3;
const double rad_to_deg = 57.2957795; // 57.2957795
const double deg_to_rad = 0.0174532925; // 0.0174532925
double wheel_diameter = 6.8; //cm 6.8  21.3
double radius_wheel = 3.4; //cm 0.034m  3.4 cm  68mm diameter
double wheel_round = 21.3; // obikolka na koleloto w cm
double width_of_car = 18.0;
const double rpm_to_radians = 0.10471975512;


const float Pi = 3.141592653589793;
double position_rotation_step[3]{ 0,0,0 }; //use this in roatation

class MatrixFunction {
public:
    void multiply_3x3__3x3(const double(&a)[N][N],
        const double(&b)[N][N], double(&c)[N][N]) {
        // multiply two matrices 3x3 and 3x3
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) {
                for (int u = 0; u < 3; u++)
                    c[i][j] += a[i][u] * b[u][j];
            }
    }

    void transpose_matrix_3x3(const double(&a)[N][N],
        double(&res)[N][N]) {
        int i, j;
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                res[i][j] = a[j][i];
            }

        }
    }

    void sum_two_matrices_3_x_3(const double(&a)[N][N],
        const  double(&b)[N][N], double(&res)[N][N]) {
        int i, j;
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                res[i][j] = a[i][j] + b[i][j];
            }

        }
    }

    void subtract_two_matrices_3x3__3x3(const double(&a)[N][N],
        const  double(&b)[N][N], double(&res)[N][N]) {
        int i, j;
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                res[i][j] = a[i][j] - b[i][j];
            }

        }
    }


    void multiply_3x3__and_3x1_(const double(&a)[N][N],
        const double(&b)[N], double(&res)[N]) {
        for (int i = 0; i < 3; i++) { // All array elements
            res[i] = 0;
            for (int j = 0; j < 3; j++) {
                res[j] += a[j][i] * b[i];
            }

        }
    }


    void sum_two_matrices_3x1_3x1(const double(&a)[N],
        const  double(&b)[N], double(&res)[N]) {
        int i;
        for (i = 0; i < 3; i++) {
            res[i] = a[i] + b[i];
        }
    }

    void subtract_two_matrices_3x1__3x1(const double(&a)[N],
        const  double(&b)[N], double(&res)[N]) {
        int i;
        for (i = 0; i < 3; i++) {
            res[i] = a[i] - b[i];
        }
    }

    void pinv_matrix_3x3(const double(&m)[N][N],
        double(&res)[N][N]) {
        double d = 0;

        //finding determinant of the matrix
        for (int i = 0; i < 3; i++)
            d = d + (m[0][i] * (m[1][(i + 1) % 3] * m[2][(i + 2) % 3] - m[1][(i + 2) % 3] * m[2][(i + 1) % 3]));

        if (d > 0)//Condition to check if the derterminat is zero or not if zero than inverse dont exists
        {
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    res[i][j] = ((m[(j + 1) % 3][(i + 1) % 3] *
                        m[(j + 2) % 3][(i + 2) % 3]) -
                        (m[(j + 1) % 3][(i + 2) % 3] *
                            m[(j + 2) % 3][(i + 1) % 3])) / d; //finding adjoint and dividing it by determinant
                }
            }

        }


    }

    void assume_matrix_3x3(const double(&from)[N][N],
        double(&to)[N][N]) {
        int i, j;
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                to[i][j] = from[i][j];

            }
        }
    }

    void assume_matrix_3x1(const double(&from)[N],
        double(&to)[N]) {
        int i;
        for (i = 0; i < 3; i++) {
            to[i] = from[i];
            //cout:fixed;
            ///out << from[i] << endl;
            //printf("%0.6f\n", double(to[i]));
        }
    }









};


class KalmanSTEPnew : public MatrixFunction{
    //A matrix = > how state of the system[x, y, yaw] changes
    double STATE_CHANGES_X = 1.0;
    double STATE_CHANGES_Y = 1.0;
    double STATE_CHANGES_THETA = 0.1;  // 1.0
    // proccess noice
    double PROCCESS_NOICE_X = 0.01;  // 0.01
    double PROCCESS_NOICE_Y = 0.01;  // 0.01
    double PROCCESS_NOICE_THETA = 0.003; // 0.003
    // Q matrix = > more Q in sensor, small Q prediction
    double STATE_MODEL_NOICE_X = 1.0;
    double STATE_MODEL_NOICE_Y = 1.0; //1
    double STATE_MODEL_NOICE_THETA = 1.0;
    // H identity matrix Measurement matrix, convert the predicted state = > measurments state
    double IDENTITY_X = 1.0;
    double IDENTITY_Y = 1.0;
    double IDENTITY_THETA = 1.0;

    // R Sensor measurement noise covariance, if sure in R, R = 0
    double SENSOR_MEASURMENT_NOICE_X = 1.0;
    double SENSOR_MEASURMENT_NOICE_Y = 1.0;
    double SENSOR_MEASURMENT_NOICE_THETA = 1.0;  // 1.0
    // Sensor noice
    double SENSOR_NOICE_X = 0.00004; // - 0.04
    double SENSOR_NOICE_Y = 0.0048; //0.049, 0.042
    double SENSOR_NOICE_THETA = -0.1;  //  + 0.0027,-0.1  which direction make error + less angle, - more

    // estimation state, where the robot start
    double ESTIMATET_STATE_LAST_X = 0;
    double ESTIMATET_STATE_LAST_Y = 0;
    double ESTIMATET_STATE_LAST_THETA = 0; //radians
    double state_estimate_k_minus_1[3] = { ESTIMATET_STATE_LAST_X,
        ESTIMATET_STATE_LAST_Y,
        ESTIMATET_STATE_LAST_THETA }; // [meters, meters, radians]


    // P matrix
    double ACCURACY_STATE_X = 0.1;
    double ACCURACY_STATE_Y = 0.1;
    double ACCURACY_STATE_THETA = 0.1; // 0.1

    double ACCURACY_STATE_X_curr = 0.1;
    double ACCURACY_STATE_Y_curr = 0.1;
    double ACCURACY_STATE_THETA_curr = 0.1; // 0.1

    double P_k_minus_1[3][3] = { {ACCURACY_STATE_X, 0, 0},
                                {0, ACCURACY_STATE_Y, 0},
                                {0, 0, ACCURACY_STATE_THETA} };


    ////////////////////////////////////////////////////////////////////
    double A_k_minus_1[3][3] = {
                {STATE_CHANGES_X, 0, 0},
                { 0, STATE_CHANGES_Y, 0},
                {0, 0, STATE_CHANGES_THETA} };
    double process_noise_v_k_minus_1[3] = { PROCCESS_NOICE_X,
                                            PROCCESS_NOICE_Y,
                                            PROCCESS_NOICE_THETA };
    double Q_k[3][3] = { {STATE_MODEL_NOICE_X, 0, 0},
                       {0, STATE_MODEL_NOICE_Y, 0},
                       {0, 0, STATE_MODEL_NOICE_THETA} };

    double H_k[3][3] = { {IDENTITY_X, 0, 0},
                        {0, IDENTITY_Y, 0 },
                         {0, 0, IDENTITY_THETA} };

    double R_k[3][3] = { {SENSOR_MEASURMENT_NOICE_X, 0, 0},
                        {0, SENSOR_MEASURMENT_NOICE_Y, 0},
                        {0, 0, SENSOR_MEASURMENT_NOICE_THETA} };

    double sensor_noise_w_k[3] = { SENSOR_NOICE_X,
                    SENSOR_NOICE_Y,
                    SENSOR_NOICE_THETA };

    double R_matrix[3][3] = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };
    double prediction_ekf[3] = { 0,0,0 };

    double final_P_k[3][3] = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };
    double state_estimate_k_updated_step[3] = { 0,0,0 };



    void ekf(double(&z_k_observation_vector_re)[3],
        double(&state_estimate_k_minus_1_re)[3],       
        double(&P_k_minus_1_re)[3][3]
        ) {
     
        ////// PREDICT /////////////////////////
        double state_estimate_k_prediction[3] = { 0,0,0 };
        for (int i = 0; i < 3; i++) {
            state_estimate_k_prediction[i] = z_k_observation_vector_re[i] +
                process_noise_v_k_minus_1[i];

        }

        //Predict the state covariance
        // P_k = self.A_k_minus_1 @ P_k_minus_1 @ self.A_k_minus_1.T + (self.Q_k)
        // A_k_minus_1[3][3],P_k_minus_1[3][3] @ A_k_minus_1.T[3][3] + self.Q_k[3][3]
        double P_k_1[3][3] = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };;

        multiply_3x3__3x3(A_k_minus_1, P_k_minus_1, P_k_1);
        double P_k_2[3][3] = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };;
        double A_k_minus_1_T[3][3] = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };;
        transpose_matrix_3x3(A_k_minus_1, A_k_minus_1_T);
        multiply_3x3__3x3(P_k_1, A_k_minus_1_T, P_k_2);
        double P_k[3][3] = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };; // final
        sum_two_matrices_3_x_3(P_k_2, Q_k, P_k);
        //print_3x3_matrix(P_k);


        ////////////////   Update (Correct) ////////////////////////
        //= z_k_observation_vector[3] - ((self.H_k @ state_estimate_k_prediction) + (self.sensor_noise_w_k))
        double measurement_residual_y_k_1[3] = { 0,0,0 };
        double meas_res_1[3] = { 0.0,0.0,0.0 };
        multiply_3x3__and_3x1_(H_k, state_estimate_k_prediction, meas_res_1);
        sum_two_matrices_3x1_3x1(meas_res_1, sensor_noise_w_k, measurement_residual_y_k_1);
        double measurement_residual_y_k[3] = { 0,0,0 };
        subtract_two_matrices_3x1__3x1(z_k_observation_vector_re, measurement_residual_y_k_1, measurement_residual_y_k);


        //print_3x1_matrix(measurement_residual_y_k);

        //Calculate the measurement residual covariance
        //S_k = self.H_k[3][3] @ P_k[3][3] @ self.H_k.T[3][3] + self.R_k[3][3]
        double S_k_1[3][3] = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };
        multiply_3x3__3x3(H_k, P_k, S_k_1);
        double S_k_2[3][3] = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };
        double H_k_Transpose[3][3] = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };
        transpose_matrix_3x3(H_k, H_k_Transpose);
        multiply_3x3__3x3(S_k_1, H_k_Transpose, S_k_2);
        double S_k[3][3] = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };
        sum_two_matrices_3_x_3(S_k_2, R_k, S_k);

        //Calculate the near-optimal Kalman gain
        //K_k = P_k @ self.H_k.T @ np.linalg.pinv(S_k)

        double K_k_1[3][3] = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };
        multiply_3x3__3x3(P_k, H_k_Transpose, K_k_1);
        double K_k[3][3] = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };
        double pinv_S_k[3][3] = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };
        pinv_matrix_3x3(S_k, pinv_S_k);
        multiply_3x3__3x3(K_k_1, pinv_S_k, K_k);

        // Calculate an updated state estimate for time k
        //  state_estimate_k_updated = state_estimate_k_prediction + 
        //   (K_k @ measurement_residual_y_k)

        //double state_estimate_k_updated[3];
        double stat_midle[3] = { 0,0,0 };
        multiply_3x3__and_3x1_(K_k, measurement_residual_y_k, stat_midle);
        sum_two_matrices_3x1_3x1(state_estimate_k_prediction, stat_midle, state_estimate_k_updated_step);

        // Update the state covariance estimate for time k
        // P_k = P_k - (K_k @ self.H_k @ P_k)
        double help_b[3][3] = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };
        double help_a[3][3] = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };
        multiply_3x3__3x3(K_k, H_k, help_a);
        multiply_3x3__3x3(help_a, P_k, help_b);
        //double final_P_k[3][3];
        subtract_two_matrices_3x3__3x3(P_k, help_b, final_P_k);



        // Return the updated state and covariance estimates
        // return state_estimate_k_updated, P_k //out of this scope

        assume_matrix_3x1(state_estimate_k_updated_step, state_estimate_k_minus_1_re);
        assume_matrix_3x3(final_P_k, P_k_minus_1_re);

        ROBOT_X_STEP = state_estimate_k_minus_1_re[0];
        ROBOT_Y_STEP = state_estimate_k_minus_1_re[1];
        ROBOT_THETA_STEP = state_estimate_k_minus_1_re[2];

    }


public:
    
    void calculation_step(double(&z_k)[3],double V_l, double V_r) {
        
        //calculate into final_P_k ,state_estimate_k_updated
        if (V_l != 0 || V_r != 0) {
            ekf(z_k, state_estimate_k_minus_1,
                P_k_minus_1);

        }

    }


    void restart_calculation() {


        for (int i = 0; i < N; i++) {
            state_estimate_k_minus_1[i] = 0;
        }
        double  reset_P_k_minus_1[3][3] = { { ACCURACY_STATE_X, 0, 0 },
             {0, ACCURACY_STATE_Y, 0},
             {0, 0, ACCURACY_STATE_THETA} };

        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                P_k_minus_1[i][j] = reset_P_k_minus_1[i][j];
            }
        }

        ROBOT_X_STEP = 0;
        ROBOT_Y_STEP = 0;
        ROBOT_THETA_STEP = 0;
    }


};


void calculation_position_step(double vel_l, double vel_r,
    double delta_time, double curr_x, double curr_y, double curr_theta) {

    double vel_L = vel_l * rpm_to_radians;
    double vel_R = vel_r * rpm_to_radians;
    //curr_theta = curr_theta * (3.1415 / 180);;
    double omega = (((vel_L - vel_R) / (width_of_car)) * radius_wheel) * delta_time;

    double R = 0;
    if (vel_L != vel_R) {
        R = width_of_car / 2.0 * ((vel_R + vel_L) / (vel_L - vel_R));
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
    double result[3] = { 0.0,0.0,0.0 };
    //calculation the matrix

    for (int i = 0; i < 3; i++) { // All array elements
        result[i] = 0;
        for (int j = 0; j < 3; j++) {
            result[i] += R_matrix[j][i] * A[i];
        }

    }




    position_rotation_step[0] = result[0] + B[0];
    position_rotation_step[1] = result[1] + B[1];
    position_rotation_step[2] = result[2] + B[2];


    cout << "== x ===== y ===== theta ==  " << endl;
    cout << position_rotation_step[0] << " , " << position_rotation_step[1]
        << " , " << position_rotation_step[2] * rad_to_deg << endl;



    

    /*ROBOT_X_STEP = position_rotation_step[0];
    ROBOT_Y_STEP = position_rotation_step[1];
    ROBOT_THETA_STEP = position_rotation_step[2];*/

    ////// kalman ///////
    KalmanSTEPnew ekf_step;
    ekf_step.calculation_step(position_rotation_step, vel_L, vel_R);

}

    


/////////////////////////////////////////////////

int main(){    
    double v_l = 20;
    double v_r = -20.00001;
    double tim = 1; //2=90,1.92=90

    for (int i = 0; i < 3;i++) {
        calculation_position_step(v_l, v_r, tim, ROBOT_X_STEP, ROBOT_Y_STEP, ROBOT_THETA_STEP);

        cout << ROBOT_X_STEP << " , " << ROBOT_Y_STEP << " , " << ROBOT_THETA_STEP * rad_to_deg << endl;

    }
             
    return 0;
}

