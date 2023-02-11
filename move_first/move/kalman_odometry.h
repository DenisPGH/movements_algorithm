#include <cmath>
#include<iostream>
//#include<algorithm>
using namespace std;
const int N=3;

class KalmanVariablesOdometry {
public:

    double CONTROL_YAW_RATE = 0.0;  // rad / sec
    int WIDTH_CAR = 18;
    double RADIUS_WHEEL = 3.4;
    double rpm_to_radians = 0.10471975512;

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
    double SENSOR_MEASURMENT_NOICE_THETA = 1;  // 1.0
    // Sensor noice
    double SENSOR_NOICE_X = -0.04; // - 0.04
    double SENSOR_NOICE_Y = 0.048; //0.049, 0.042
    double SENSOR_NOICE_THETA = 0.0027;  //  + 0.0026, -0.004 which direction make noise

    // estimation state, where the robot start
    double ESTIMATET_STATE_LAST_X = 0;
    double ESTIMATET_STATE_LAST_Y = 0;
    double ESTIMATET_STATE_LAST_THETA = 0.0; //radians
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


   
};



class KalmanOdometry : public KalmanVariablesOdometry {
    double control_vector_k_minus_1[3] = { 0, 0, CONTROL_YAW_RATE };
    double control_vector_k_minus_1_step[3] = { 0, 0, CONTROL_YAW_RATE };
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
                         {0, 0, IDENTITY_THETA}};

    double R_k[3][3] = { {SENSOR_MEASURMENT_NOICE_X, 0, 0},
                        {0, SENSOR_MEASURMENT_NOICE_Y, 0},
                        {0, 0, SENSOR_MEASURMENT_NOICE_THETA} };

    double sensor_noise_w_k[3] = { SENSOR_NOICE_X,
                    SENSOR_NOICE_Y,
                    SENSOR_NOICE_THETA };

    double R_matrix[3][3];
    double prediction_ekf[3] = { 0,0,0 };

    

private:
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

    void print_3x3_matrix(const double(&m)[N][N]) {
        int i, j;
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                cout << m[i][j]<< " ";

            }
            cout << endl;
        }


    }


    void print_3x1_matrix(const double(&m)[N]) {
        int i;
        for (i = 0; i < 3; i++) {
              cout << m[i] << " ";  
        }
        cout << endl;


    }



    void get_B(double control_vector_k_minus_1[3],
        double state_estimate_k_minus_1[3], double dt) {
        /*  :param control_vector_k_minus_1 : [rpm, rpm, rad / sec]
            : param state_estimate_k_minus_1 : [cm, cm, rad]
            : return : prediction in world frame
            """*/

        double vel_l = control_vector_k_minus_1[0] * rpm_to_radians;
        double vel_r = control_vector_k_minus_1[1] * rpm_to_radians;
        double omega = (((vel_l - vel_r) / (WIDTH_CAR)) * RADIUS_WHEEL) * dt;
        double R = 0;
        if (vel_l != vel_r) {
            R = WIDTH_CAR / 2.0 * ((vel_r + vel_l) / (vel_l - vel_r));
        }
        else { R = 0.0; }


        double ICC_x = state_estimate_k_minus_1[0] - R * sin(state_estimate_k_minus_1[2]);
        double ICC_y = state_estimate_k_minus_1[1] + R * cos(state_estimate_k_minus_1[2]);

        double R_matrix[3][3] = { {cos(omega), -sin(omega), 0},
                                 {sin(omega), cos(omega), 0},
                                 {0, 0, 1},
        };

        double A[3] = { state_estimate_k_minus_1[0] - ICC_x,
                        state_estimate_k_minus_1[1] - ICC_y,
                            state_estimate_k_minus_1[2] };

        double B[3] = { ICC_x, ICC_y, omega };


        //float result[3] = R_matrix @ A + B.T;
        double result[3] = { 0,0,0 };
        //calculation the matrix

        for (int i = 0; i < 3; i++) { // All array elements
            result[i] = 0;
            for (int j = 0; j < 3; j++) {
                result[j] += R_matrix[j][i] * A[i];
            }

        }


        prediction_ekf[0] = result[0] + B[0];
        prediction_ekf[1] = result[1] + B[1];
        prediction_ekf[2] = result[2] + B[2];
        
        
    }



    void ekf(double(&z_k_observation_vector)[3],
        double state_estimate_k_minus_1[3],
        double control_vector_k_minus_1[3],
        double(&P_k_minus_1)[3][3],
        double dk) {

        //print_3x1_matrix(z_k_observation_vector);

        ////// PREDICT /////////////////////////
        double state_estimate_k_prediction[3] = {0,0,0};
        get_B(control_vector_k_minus_1, state_estimate_k_minus_1, dk);
        for (int i = 0; i < 3; i++) {
            state_estimate_k_prediction[i] = prediction_ekf[i] +
                process_noise_v_k_minus_1[i];

        }
        
        //Predict the state covariance
        // P_k = self.A_k_minus_1 @ P_k_minus_1 @ self.A_k_minus_1.T + (self.Q_k)
        // A_k_minus_1[3][3],P_k_minus_1[3][3] @ A_k_minus_1.T[3][3] + self.Q_k[3][3]
        double P_k_1[3][3]= { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };;
       
        multiply_3x3__3x3(A_k_minus_1, P_k_minus_1, P_k_1);
        double P_k_2[3][3]={ {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };;
        double A_k_minus_1_T[3][3]= { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };;
        transpose_matrix_3x3(A_k_minus_1, A_k_minus_1_T);
        multiply_3x3__3x3(P_k_1, A_k_minus_1_T, P_k_2);
        double P_k[3][3]= { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };; // final
        sum_two_matrices_3_x_3(P_k_2, Q_k, P_k);
        //print_3x3_matrix(P_k);
        

        ////////////////   Update (Correct) ////////////////////////
        //= z_k_observation_vector[3] - ((self.H_k @ state_estimate_k_prediction) + (self.sensor_noise_w_k))
        double measurement_residual_y_k_1[3] = {0,0,0};
        double meas_res_1[3] = {0.0,0.0,0.0};
        multiply_3x3__and_3x1_(H_k, state_estimate_k_prediction, meas_res_1);
        sum_two_matrices_3x1_3x1(meas_res_1, sensor_noise_w_k, measurement_residual_y_k_1);
        double measurement_residual_y_k[3] = { 0,0,0 };
        subtract_two_matrices_3x1__3x1(z_k_observation_vector, measurement_residual_y_k_1, measurement_residual_y_k);
        
       
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
        sum_two_matrices_3x1_3x1(state_estimate_k_prediction, stat_midle, state_estimate_k_updated);

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

    }


public:
    // after ekf 
    double final_P_k[3][3] = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };
    double state_estimate_k_updated[3]= {0,0,0};

    void calculation_ekf(double (&z_k)[3],double dt, double V_l,double V_r ) {
        double control_vector_k_minus_1[3] = { V_l, V_r,CONTROL_YAW_RATE }; // [rpm, rpm, rad / sec]

        //calculate into final_P_k ,state_estimate_k_updated
        ekf(z_k, state_estimate_k_minus_1, control_vector_k_minus_1,
            P_k_minus_1, dt);

        //update 
        //state_estimate_k_minus_1 = state_estimate_k_updated
         //P_k_minus_1 = final_P_k
        assume_matrix_3x1(state_estimate_k_updated, state_estimate_k_minus_1);
        assume_matrix_3x3(final_P_k, P_k_minus_1);

    }

   

  
    

    


    

};