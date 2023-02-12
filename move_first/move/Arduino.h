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