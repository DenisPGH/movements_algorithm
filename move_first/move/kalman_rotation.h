#include <iostream>
using namespace std;


class KalmanRotation {

    double deg_to_rad = 0.0174532925;
    double rad_to_deg = 57.29578;

    double STATE_CHANGES = 1; // 1.0
    double PROCCESS_NOICE = 0.1; //  # 0.01
    double STATE_MODEL_NOICE = 1.0;
    double IDENTITY = 1.0;
    double SENSOR_MEASURMENT_NOICE = 0.006; // # 0.0001
    double SENSOR_NOICE = -0.006; //#  - 0.06
    double ESTIMATET_STATE_LAST = 0; //math.radians(0) # always from 0 deg
    double SPEED = 1.0;  //deg / sec
    double ACCURACY_STATE = 0.003; //  0.3
    /////////////////////////////////////////////
    double A_k_minus_1 = { STATE_CHANGES };
    double process_noise_v_k_minus_1 = { PROCCESS_NOICE };
    double Q_k = { STATE_MODEL_NOICE };
    double H_k = { IDENTITY };
    double R_k = { SENSOR_MEASURMENT_NOICE };
    double sensor_noise_w_k = { SENSOR_NOICE };
    double state_estimate_k_minus_1 = { ESTIMATET_STATE_LAST };  // [radians], start position
    double control_vector_k_minus_1 = { SPEED * deg_to_rad }; //rad
    double P_k_minus_1 = { ACCURACY_STATE };
    double end_position = 0;

    double state_estimate_k = 0;
    double P_k_end = 0;

public:
    void restart_calculation() {
        cout << "restart calc rot" << endl;
    }

    double getB(double delta_theta) {
        double B = { delta_theta * deg_to_rad };
        return B;
    }

    double inverse_double(double d) {
        //np.linalg.pinv inverse of matrix 1x1
        return (1/d);
    }

    void ekf_rotation(double z_k_observation_vector, double state_estimate_k_minus_1,
        double control_vector_k_minus_1,double  P_k_minus_1) {
        // predict step
        double state_estimate_k_ = A_k_minus_1 * (state_estimate_k_minus_1)+(getB(state_estimate_k_minus_1)) * (
            control_vector_k_minus_1)+(process_noise_v_k_minus_1);

        double P_k = A_k_minus_1 * P_k_minus_1 * A_k_minus_1 + (Q_k);
        //# update
        double measurement_residual_y_k = z_k_observation_vector - ((H_k * state_estimate_k_) + (sensor_noise_w_k));
        //Calculate the measurement residual covariance
        double S_k = H_k * P_k * H_k + R_k;
        double K_k = P_k * H_k * inverse_double(S_k);
        state_estimate_k = state_estimate_k_ + (K_k * measurement_residual_y_k);
        P_k_end = P_k - (K_k * H_k * P_k);
        double after = state_estimate_k * rad_to_deg;
        end_position = after;
        
    }

	double calculation_error_rotation(double z_k) {
		/// <summary>
		/// 
		/// </summary>
		/// <param name="angle"> radians </param>
		/// <returns></returns>
        /// 
        ekf_rotation(z_k * deg_to_rad,
            state_estimate_k_minus_1, control_vector_k_minus_1,
            P_k_minus_1);

        //update matrices
        state_estimate_k_minus_1 = state_estimate_k;
        P_k_minus_1 = P_k_end;
		return end_position;
	}
};
