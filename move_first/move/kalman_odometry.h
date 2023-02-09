#include <cmath>

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

    double state_estimate_k_minus_1_current_step[3] = { 0, 0, 0 };
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


    double P_k_minus_1_current_step[3][3] = { {ACCURACY_STATE_X_curr, 0, 0},
                                 { 0, ACCURACY_STATE_Y_curr, 0 },
                                    {0, 0, ACCURACY_STATE_THETA_curr} };
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


    


public:

    void multiply_3_x_3_matrix(double a[3][3], double b[3][3], double c[3][3]) {
        // multiply two matrices 3x3 and 3x3
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) {
                for (int u = 0; u < 3; u++)
                    c[i][j] += a[i][u] * b[u][j];
            }
    }

    void get_B(double control_vector_k_minus_1[3], 
        double state_estimate_k_minus_1[3], double dt) {
        /*  :param control_vector_k_minus_1 : [rpm, rpm, rad / sec]
            : param state_estimate_k_minus_1 : [cm, cm, rad]
            : return : prediction in world frame
            """*/

        double vel_l=control_vector_k_minus_1[0] * rpm_to_radians;
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



    void ekf(double z_k_observation_vector[3],
        double state_estimate_k_minus_1[3],
        double control_vector_k_minus_1[3],
        double P_k_minus_1[3][3],
        double dk) {

        ////// PREDICT /////////////////////////
        double state_estimate_k_prediction[3];
        for (int i=0; i < 3; i++) {
            state_estimate_k_prediction[i] = prediction_ekf[i] +
                process_noise_v_k_minus_1[i];

        }
        //Predict the state covariance
        // P_k = self.A_k_minus_1 @ P_k_minus_1 @ self.A_k_minus_1.T + (self.Q_k)
        // A_k_minus_1[3][3],P_k_minus_1[3][3] @ A_k_minus_1.T[3][3] + self.Q_k[3][3]
        double P_k[3][3];
    


        
           
    }


    

};