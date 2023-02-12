#include <chrono>
using namespace std::chrono;


float speedRateL = 0;
float speedRateR = 0;
//float interval_delta_time = 0.4;
double interval_delta_time = duration<double>(1).count();



milliseconds millis() {
	/// <summary>
	/// like a millis in arduino
	/// </summary>
	/// <returns></returns>
	milliseconds ms = duration_cast<milliseconds>(
		system_clock::now().time_since_epoch()
		);
	
	return ms;
}

milliseconds double_to_chrono(double d) {
	milliseconds res = duration_cast<milliseconds>(
		duration<double>{(d)});
	return res;
}

void moving_PWM(float outPWM_L, float outPWM_R) {
	cout << outPWM_L << " DC " << outPWM_R << endl;
}


class ControlMotors {
public:

	void movement(double pwd_l,double pwd_r, int direction) {
		/// <summary>
		/// 
		/// </summary>
		/// <param name="pwd_l"> -255 do 255</param>
		/// <param name="pwd_r">-255 do 255</param>
		/// <param name="direction"> L,R, Ahead, Back</param>
		moving_PWM(pwd_l, pwd_r);

	}
};