

double TARGET_ANGLE_ROTATION = 0; // main variable

class PID_Rotation {
public:
	double PID_ROTATION_OUTPUT_L = 0;
	double  PID_ROTATION_OUTPUT_R = 0;

	double Kp_L_rot = 2.9; // # 10.62
	double Ki_L_rot = 0; // # 0
	double Kd_L_rot = 0; // # 0

	double Kp_R_rot = 2.9; // # 9.2
	double Ki_R_rot = 0; //  # 0
	double Kd_R_rot = 0; // # 0

	double setpoint_rot_L = 0;
	double setpoint_rot_R = 0;

	void config_pid_rotation() {
		/// <summary>
		/// have to run before setup
		/// </summary>
		PID ROTATION_PID_L(&TARGET_ANGLE_ROTATION, &PID_ROTATION_OUTPUT_L, &setpoint_rot_L, Kp_L_rot, Ki_L_rot, Kd_L_rot, DIRECT);
		PID ROTATION_PID_R(&TARGET_ANGLE_ROTATION, &PID_ROTATION_OUTPUT_R, &setpoint_rot_R, Kp_R_rot, Ki_R_rot, Kd_R_rot, DIRECT);
	}

	void setup_pid_rotation() {
		ROTATION_PID_L.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
		ROTATION_PID_L.SetMode(AUTOMATIC);
		ROTATION_PID_L.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI_L);

		ROTATION_PID_R.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
		ROTATION_PID_R.SetMode(AUTOMATIC);
		ROTATION_PID_R.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI_R);
		}


	void output_rotation() {
		/// <summary>
		/// use instance.PID_ROTATION_OUTPUT_L and PID_ROTATION_OUTPUT_R
		/// </summary>

		ROTATION_PID_L.Compute(); // calculate PID 
		ROTATION_PID_R.Compute(); // calculate PID 
		

	}
};
