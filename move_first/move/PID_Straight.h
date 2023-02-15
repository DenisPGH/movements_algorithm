
#define PID_MIN_LIMIT -255  //Min limit for PID 
#define PID_MAX_LIMIT 255  //Max limit for PID 

#define PID_SAMPLE_TIME_IN_MILLI_L 10  //This is PID sample time in milliseconds 10
#define PID_SAMPLE_TIME_IN_MILLI_R 10  //This 


double ROBOT_THETA_ROTATION = 0; //radians
double rpm_L = 0;
double rpm_R = 0;



class PID_Straight {
public:
    double Kp_L_theta = 30; // # 1.9
    double Ki_L_theta = 100; //  # 0
    double Kd_L_theta= 1 ; // # 0

    double Kp_R_theta = 30 ; // # 1.9
    double Ki_R_theta = 100 ; // # 0
    double Kd_R_theta = 1 ; // # 0

    double final_speed_l = 0; //
    double final_speed_r = 0; //

    double setpoint_theta = 0;

    double theta_pid_output_l = 0; // output PID_L
    double theta_pid_output_r = 0; // output PID_R  kakvo chislo otiva kym motori
	//@@@@@@@@@///

    double Kp_L = 2.9; // # 10.62
    double Ki_L = 0 ; // # 0
    double Kd_L = 0; // # 0

    double Kp_R = 2.9; // # 9.2
    double Ki_R = 0 ; //  # 0
    double Kd_R = 0; // # 0

    double SPEED_L_target = 20; // #20
    double SPEED_R_target = 20; //

    double PID_STRAIGHT_OUTPUT_L = 0; // output to motors
    double PID_STRAIGHT_OUTPUT_R = 0; // out

    void config_pid_straght() {
        /// <summary>
        /// have to run before setup
        /// </summary>
        PID THETA_PID_L(&ROBOT_THETA_ROTATION, &theta_pid_output_l, &setpoint_theta, Kp_L_theta, Ki_L_theta, Kd_L_theta, DIRECT);
        PID THETA_PID_R(&ROBOT_THETA_ROTATION, &theta_pid_output_r, &setpoint_theta, Kp_R_theta, Ki_R_theta, Kd_R_theta, DIRECT);
        

        PID STRAIGHT_SPEED_PID_L(&rpm_L, &PID_STRAIGHT_OUTPUT_L, &final_speed_l, Kp_L, Ki_L, Kd_L, DIRECT);
        PID STRAIGHT_SPEED_PID_R(&rpm_R, &PID_STRAIGHT_OUTPUT_R, &final_speed_r, Kp_R, Ki_R, Kd_R, DIRECT);
    
    
    
    }


    void setup_pid_straight() {
        /// <summary>
        /// run in setup of the arduino
        /// </summary>

        THETA_PID_L.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
        THETA_PID_L.SetMode(AUTOMATIC);
        THETA_PID_L.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI_L);

        THETA_PID_R.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
        THETA_PID_R.SetMode(AUTOMATIC);
        THETA_PID_R.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI_R);

        STRAIGHT_SPEED_PID_L.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
        STRAIGHT_SPEED_PID_L.SetMode(AUTOMATIC);
        STRAIGHT_SPEED_PID_L.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI_L);

        STRAIGHT_SPEED_PID_R.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
        STRAIGHT_SPEED_PID_R.SetMode(AUTOMATIC);
        STRAIGHT_SPEED_PID_R.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI_R);

    }

	void output_straight_line() {
        /// <summary>
        /// calculate the output values and apply to the motors
        /// run this in loop with control use instance.PID_STRAIGHT_OUTPUT_L
        /// </summary>

        THETA_PID_L.Compute(); // calculate PID theta error
        THETA_PID_R.Compute(); // calculate PID theta error

        if (ROBOT_THETA_ROTATION == 0) {
            final_speed_l = SPEED_L_target;
            final_speed_r = SPEED_R_target;

        }
            
        else if (ROBOT_THETA_ROTATION < 0) {
            final_speed_l = SPEED_L_target - theta_pid_output_l;
            final_speed_r = SPEED_R_target + theta_pid_output_r;

        }

        else if (ROBOT_THETA_ROTATION > 0) {
            final_speed_l = SPEED_L_target + theta_pid_output_l;
            final_speed_r = SPEED_R_target - theta_pid_output_r;

        }

        STRAIGHT_SPEED_PID_L.Compute(); // calculate PID final
        STRAIGHT_SPEED_PID_R.Compute(); // 
            
	}
};
