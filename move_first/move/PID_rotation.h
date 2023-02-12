
class PID_Rotation {
public:
	double left_motor_output = 0;
	double right_motor_output = 0;

	void output() {
		left_motor_output = 20;
		right_motor_output = 20;

	}
};
