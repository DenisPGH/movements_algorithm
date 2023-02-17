double ROBOT_X = 0; //main vaiable
double ROBOT_Y = 0;//main vaiable
double ROBOT_THETA = 0; //radians main vaiable 
double ROBOT_THETA_ROTATION = 0; // radians //main vaiable
double ROBOT_X_ROTATION = 0; // 
double ROBOT_Y_ROTATION = 0; // 
double TARGET_ANGLE_ROTATION = 0; // main variable

double ROBOT_CURRENT_TRAVELED_DISTANCE = 0;
double ROBOT_LAST_TRAVELED_DISTANCE = 0;

bool ROBOT_ARRIVED = false;

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

#include <PID_v1.h>


const int MIN_L = 27; //30
const int MIN_R = 30; //30

const double Kp_L_theta = 30; // # 1.9
const double Ki_L_theta = 100; //  # 0
const double Kd_L_theta = 1; // # 0

const double Kp_R_theta = 30; // # 1.9
const double Ki_R_theta = 100; // # 0
const double Kd_R_theta = 1; // # 0

const double Kp_L = 2.9; // # 10.62
const double Ki_L = 0; // # 0
const double Kd_L = 0; // # 0

const double Kp_R = 2.9; // # 9.2
const double Ki_R = 0; //  # 0
const double Kd_R = 0; // # 0

const double Kp_L_rot = 2.9; // # 10.62
const double Ki_L_rot = 0; // # 0
const double Kd_L_rot = 0; // # 0

const double Kp_R_rot = 2.9; // # 9.2
const double Ki_R_rot = 0; //  # 0
const double Kd_R_rot = 0; // # 0

double MIN_ABSOLUTE_SPEED_L = MIN_L; //30 motor Left
double MIN_ABSOLUTE_SPEED_R = MIN_R; //30 motor Right

double rpm_L = 0;
double rpm_R = 0;

#define PID_MIN_LIMIT -255  //Min limit for PID 
#define PID_MAX_LIMIT 255  //Max limit for PID 

#define PID_SAMPLE_TIME_IN_MILLI_L 10  //This is PID sample time in milliseconds 10
#define PID_SAMPLE_TIME_IN_MILLI_R 10  //This is PID sample time in milliseconds 10





#include <Wire.h>
unsigned long start = 0;




unsigned long intervalTime = 400; //dt=0.4 ms
float speed_time_factor = intervalTime / 1000; //0.4

const int DIR_AHEAD = 1;
const int DIR_BACK = 2;
const int DIR_STOP = 0; //stop


unsigned long cur = 0;
unsigned long prev = 0;
unsigned long cur_bat = 0;
unsigned long prev_bat = 0;

int battery;
double BAT = 0; //real value of the battery 2
double bat_factor_pid = 0;  //value for pid
int bat_value = 0;
double BAT_percent = 0;

int oko = 11; // dqsno oko
int pin_battery = 7; // pin input status batery 12.6V

////////////////////////////////////

bool dir_speed_L = true;
bool dir_speed_R = true;
int dir_speed_L_perm = 0;
int dir_speed_R_perm = 0;

int dist_or_speed = 0; // 0=dist, 1=speed
double dist_cm_L = 0;
double dist_cm_R = 0;
////////// speed calculation v2 /////////////////////

double ang_velocity_L = 0;
double ang_velocity_deg_L = 0;
double ang_velocity_R = 0;
double ang_velocity_deg_R = 0;

const float rpm_to_radians = 0.10471975512;
const float rad_to_deg = 57.2957795; // 57.2957795

const float Pi = 3.141592653589793;

int BAT_MEASSURING_INTERVAL = 1500;

// have to store the distance, because the spead del it
unsigned long sum_pulses_L = 0;
unsigned long sum_pulses_R = 0;
unsigned long current_tics_L = 0;
unsigned long current_tics_R = 0;


// variables for calculating the theta angle-orientation
double speed_L_teta = 0;
double speed_R_teta = 0;
int wheel_diameter = 6.8; //cm 6.8  21.3
double radius_wheel = 3.4; //cm 0.034m  3.4 cm  68mm diameter
double wheel_round = 21.3; // obikolka na koleloto w cm




double final_speed_l = 0; //
double final_speed_r = 0; //

double setpoint_theta = 0;

double theta_pid_output_l = 0; // output PID_L
double theta_pid_output_r = 0; // output PID_R  kakvo chislo otiva kym motori



const double SPEED = 20;
double SPEED_L_target = SPEED; // #20
double SPEED_R_target = SPEED; //

double PID_STRAIGHT_OUTPUT_L = 0; // output to motors
double PID_STRAIGHT_OUTPUT_R = 0; // out

double PID_ROTATION_OUTPUT_L = 0;
double  PID_ROTATION_OUTPUT_R = 0;



double setpoint_rot_L = 0;
double setpoint_rot_R = 0;

PID ROTATION_PID_L(&TARGET_ANGLE_ROTATION, &PID_ROTATION_OUTPUT_L, &setpoint_rot_L, Kp_L_rot, Ki_L_rot, Kd_L_rot, DIRECT);
PID ROTATION_PID_R(&TARGET_ANGLE_ROTATION, &PID_ROTATION_OUTPUT_R, &setpoint_rot_R, Kp_R_rot, Ki_R_rot, Kd_R_rot, DIRECT);


PID THETA_PID_L(&ROBOT_THETA_ROTATION, &theta_pid_output_l, &setpoint_theta, Kp_L_theta, Ki_L_theta, Kd_L_theta, DIRECT);
PID THETA_PID_R(&ROBOT_THETA_ROTATION, &theta_pid_output_r, &setpoint_theta, Kp_R_theta, Ki_R_theta, Kd_R_theta, DIRECT);


PID STRAIGHT_SPEED_PID_L(&rpm_L, &PID_STRAIGHT_OUTPUT_L, &final_speed_l, Kp_L, Ki_L, Kd_L, DIRECT);
PID STRAIGHT_SPEED_PID_R(&rpm_R, &PID_STRAIGHT_OUTPUT_R, &final_speed_r, Kp_R, Ki_R, Kd_R, DIRECT);

///////////////////////////////// PID Speed ///////////////////////////////////////////////



volatile long curSpeedL = 0;
volatile long curSpeedR = 0;
volatile long prevSpeedL = 0;
volatile long prevSpeedR = 0;

unsigned long curTime = 0;
unsigned long prevTime = 0;
//unsigned long intervalTime = 300; // intensitet of speed calculation in ms 50

//double rps_L = 0;
//double rps_R = 0;

float leftRev = 1445;  // counts per revolution motor L 1447
float rightRev = 1441;  // counts per revolution motor R 1441

// desired speed give here

double setpointSpeedL = 20;  //20    target speed L  
double setpointSpeedR = 20;  //20   target speed R 

double speedRateL = 0;   // real value speed L
double speedPIDOutputL = 0; // output PID_L
double speedRateR = 0;   // real value speed R  kolko e v momenta
double speedPIDOutputR = 0; // output PID_R  kakvo chislo otiva kym motori






/////////////////// PID speed stop ////////////////////////////////////////////////////////



/////////////////////////Encoder ///////////////////////////////////////////
#define enc_L 3 // interupt
#define enc_L_dir 13 // posoka na vyrtene 13

#define enc_R 2 //position 2
#define enc_R_dir 12 // 12

bool directionsL = true;  // true is forward, false is backward
volatile long countPulsesL = 0;
bool directionsR = true;  // true is forward, false is backward
volatile long countPulsesR = 0;

volatile long S = 100;
unsigned long counter_a = 0;
bool flag_2 = false;

unsigned long start_encoder_L = 0;
unsigned long start_encoder_R = 0;

int funk = 0;
int direktion_L = 0;  //
int direktion_R = 0;  //
////////////////////// Encoder stop //////////////////////////////////////////////




bool flag = false;
double finalPWM_L;
double finalPWM_R;

float L, R;

///////////////////// DC ///////////////////

// positon L
int enL = 5; // A
int in_1 = 7;
int in_2 = 8;

//  positon R
int enR = 6; // B
int in_3 = 9;
int in_4 = 10;
////////////////////////// DC //////////
//////////////////////// I2c start //////////////////////////////////////////////
#define SLAVE_ADDRESS 0x60 /// random value of Arduino Nano
volatile bool receiveFlag = false;
char temp[32];
String command;
int my_array[7];
char msg[7];

int dir_jet = 0;
double jet_pid_L = 0;
double jet_pid_R = 0;
int mode = 0;

int LED_STATUS_JETSON = 0;
int DISTANCE_JETSON = 0;
int TARGET_ORIENTATION_JETSON = 0;
int DIFFERENCE_ANGLE_JETSON = 0;
char DIRECTION = 'O';

char jetson[40];
//////////////////////// I2c stop //////////////////////////////////////////////








/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////// ODOMETRY CLASS START /////////////////////////////////////////////////////////
const int N = 3;

class ODOMETRY {

    double rpm_to_radians = 0.10471975512;
    double radius_wheel = 3.4;
    double width_of_car = 18.0;

private:
    double calculation_omega(double vel_l, double vel_r, double delta_time) {
        double  omega = (((vel_l - vel_r) / (width_of_car)) * radius_wheel) * delta_time;
        return omega;
    }


public:
    void print_3x1_matrix(const double(&m)[3]) {
        int i;
        for (i = 0; i < 3; i++) {
            Serial.print(m[i]);
        }
        Serial.println();

    }

    double position_full[3]{ 0,0,0 }; //use this in odometry x,y,theta
    //double position_rotation[3]{ 0,0,0 }; //use this in roatation
    void calculation_position(double vel_l, double vel_r,
        double delta_time, double curr_x, double curr_y, double curr_theta) {
        vel_l *= rpm_to_radians;
        vel_r *= rpm_to_radians;
        //curr_theta = curr_theta * (3.1415 / 180);;
        double omega = calculation_omega(vel_l, vel_r, delta_time);

        double R = 0;
        if (vel_l != vel_r) {
            R = width_of_car / 2.0 * ((vel_r + vel_l) / (vel_l - vel_r));
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
        double result[3] = { 0,0,0 };
        //calculation the matrix

        for (int i = 0; i < 3; i++) { // All array elements
            result[i] = 0;
            for (int j = 0; j < 3; j++) {
                result[j] += R_matrix[j][i] * A[i];
            }

        }


        position_full[0] = result[0] + B[0];
        position_full[1] = result[1] + B[1];
        position_full[2] = result[2] + B[2];

        /*ROBOT_X = position_full[0];
        ROBOT_Y = position_full[1];
        ROBOT_THETA = position_full[2];*/




    }


};


class ODOMETRY_ROTATION {

    double rpm_to_radians = 0.10471975512;
    double radius_wheel = 3.4;
    double width_of_car = 18.0;

private:
    double calculation_omega(double vel_l, double vel_r, double delta_time) {
        double  omega = (((vel_l - vel_r) / (width_of_car)) * radius_wheel) * delta_time;
        return omega;
    }


public:
    void print_3x1_matrix(const double(&m)[3]) {
        int i;
        for (i = 0; i < 3; i++) {
            Serial.print(m[i]);
        }
        Serial.println();

    }


    double position_rotation[3]{ 0,0,0 }; //use this in roatation
    void calculation_position_rotation(double vel_l, double vel_r,
        double delta_time, double curr_x, double curr_y, double curr_theta) {
        /// <summary>
        /// 
        /// </summary>
        /// <param name="vel_l"></param>
        /// <param name="vel_r"></param>
        /// <param name="delta_time"></param>
        /// <param name="curr_x"></param>
        /// <param name="curr_y"></param>
        /// <param name="curr_theta"> radians</param>
        vel_l *= rpm_to_radians;
        vel_r *= rpm_to_radians;
        //curr_theta = curr_theta * (3.1415 / 180);;
        double omega = calculation_omega(vel_l, vel_r, delta_time);

        double R = 0;
        if (vel_l != vel_r) {
            R = width_of_car / 2.0 * ((vel_r + vel_l) / (vel_l - vel_r));
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
        double result[3] = { 0,0,0 };
        //calculation the matrix

        for (int i = 0; i < 3; i++) { // All array elements
            result[i] = 0;
            for (int j = 0; j < 3; j++) {
                result[j] += R_matrix[j][i] * A[i];
            }

        }




        position_rotation[0] = result[0] + B[0];
        position_rotation[1] = result[1] + B[1];
        position_rotation[2] = result[2] + B[2];

        /*ROBOT_X_ROTATION = position_rotation[0];
        ROBOT_Y_ROTATION = position_rotation[1];
        ROBOT_THETA_ROTATION = position_rotation[2];*/



    }


};

/////////////////////////// ODOMETRY CLASS STOP/////////////////////////////////////////////////////////
//////////////////////////  KALMAN STEP START ////////////////////////////////////////////////////////////
class KalmanVariablesStep {
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



class KalmanStep : public KalmanVariablesStep {
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
                         {0, 0, IDENTITY_THETA} };

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
                Serial.print(m[i][j]);

            }
            Serial.println();
        }


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
        double state_estimate_k_prediction[3] = { 0,0,0 };
        get_B(control_vector_k_minus_1, state_estimate_k_minus_1, dk);
        for (int i = 0; i < 3; i++) {
            state_estimate_k_prediction[i] = prediction_ekf[i] +
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

    }


public:
    // after ekf 
    double final_P_k[3][3] = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };
    double state_estimate_k_updated_step[3] = { 0,0,0 };

    void calculation_step(double(&z_k)[3], double dt, double V_l, double V_r) {
        double control_vector_k_minus_1[3] = { V_l, V_r,CONTROL_YAW_RATE }; // [rpm, rpm, rad / sec]

        //calculate into final_P_k ,state_estimate_k_updated
        ekf(z_k, state_estimate_k_minus_1, control_vector_k_minus_1,
            P_k_minus_1, dt);

        //update 
        //state_estimate_k_minus_1 = state_estimate_k_updated
         //P_k_minus_1 = final_P_k
        assume_matrix_3x1(state_estimate_k_updated_step, state_estimate_k_minus_1);
        assume_matrix_3x3(final_P_k, P_k_minus_1);

        ROBOT_X_ROTATION = state_estimate_k_updated_step[0];
        ROBOT_Y_ROTATION = state_estimate_k_updated_step[1];
        ROBOT_THETA_ROTATION = state_estimate_k_updated_step[2];

    }


    void restart_calculation() {
        double reset_state_estimate_k_minus_1[3] = { ESTIMATET_STATE_LAST_X,
            ESTIMATET_STATE_LAST_Y,
            ESTIMATET_STATE_LAST_THETA };

        for (int i = 0; i < N; i++) {
            state_estimate_k_minus_1[i] = reset_state_estimate_k_minus_1[i];
        }
        double  reset_P_k_minus_1[3][3] = { { ACCURACY_STATE_X, 0, 0 },
             {0, ACCURACY_STATE_Y, 0},
             {0, 0, ACCURACY_STATE_THETA} };

        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                P_k_minus_1[i][j] = reset_P_k_minus_1[i][j];
            }
        }
    }



};

//////////////////////////  KALMAN STEP STOP ///////////////////////////////////////////////////



//////////////////////////  KALMAN ODOMETRY START ///////////////////////////////////////////////////
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
                         {0, 0, IDENTITY_THETA} };

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
        double state_estimate_k_prediction[3] = { 0,0,0 };
        get_B(control_vector_k_minus_1, state_estimate_k_minus_1, dk);
        for (int i = 0; i < 3; i++) {
            state_estimate_k_prediction[i] = prediction_ekf[i] +
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
    double state_estimate_k_updated[3] = { 0,0,0 };

    void calculation_ekf(double(&z_k)[3], double dt, double V_l, double V_r) {
        double control_vector_k_minus_1[3] = { V_l, V_r,CONTROL_YAW_RATE }; // [rpm, rpm, rad / sec]

        //calculate into final_P_k ,state_estimate_k_updated
        ekf(z_k, state_estimate_k_minus_1, control_vector_k_minus_1,
            P_k_minus_1, dt);

        //update 
        //state_estimate_k_minus_1 = state_estimate_k_updated
         //P_k_minus_1 = final_P_k
        assume_matrix_3x1(state_estimate_k_updated, state_estimate_k_minus_1);
        assume_matrix_3x3(final_P_k, P_k_minus_1);

        ROBOT_X = state_estimate_k_updated[0];
        ROBOT_Y = state_estimate_k_updated[1];
        ROBOT_THETA = state_estimate_k_updated[2];

    }



};

//////////////////////////  KALMAN ODOMETRY STOP ///////////////////////////////////////////////////



/////////////////////////////////////// BODY START /////////////////////////////////////////////////////////
void actuator(int mode, double pwm_l = 0, double pwm_r = 0) {
    /// control the movements of the car
    /// 1=ahead, 2=stop, 3=left, 4=right
    /// movinig all motors ahead
    if (mode == 1) {
        direktion_L = DIR_AHEAD; direktion_R = DIR_AHEAD;
        MIN_ABSOLUTE_SPEED_L = MIN_L; MIN_ABSOLUTE_SPEED_R = MIN_R;
        moving_PWM(pwm_l, pwm_r);
    }
    // STOP 
    else if (mode == 2) {
        direktion_L = DIR_STOP; direktion_R = DIR_STOP;
        moving_PWM(0, 0); MIN_ABSOLUTE_SPEED_L = 0; MIN_ABSOLUTE_SPEED_R = 0;
        ang_velocity_L = 0; ang_velocity_R = 0; rpm_L = 0; rpm_R = 0;

    }

    // TURN LEFT
    else if (mode == 3) {
        direktion_L = DIR_BACK; direktion_R = DIR_AHEAD;
        MIN_ABSOLUTE_SPEED_L = MIN_L; MIN_ABSOLUTE_SPEED_R = MIN_R;
        moving_PWM(pwm_l, pwm_r);
    }
    // TURN RIGHT 
    else if (mode == 4) {
        direktion_L = DIR_AHEAD; direktion_R = DIR_BACK;
        MIN_ABSOLUTE_SPEED_L = MIN_L; MIN_ABSOLUTE_SPEED_R = MIN_R;
        moving_PWM(pwm_l, pwm_r);
    }



    if (LED_STATUS_JETSON == 0) { digitalWrite(oko, LOW); } // Lights on
    else if (LED_STATUS_JETSON == 1) { digitalWrite(oko, HIGH); } // lights off



}

class BODY {
    ODOMETRY odo;
    KalmanOdometry ekf_b;
    KalmanStep ekf_step;
    ODOMETRY_ROTATION odo_rot;
    double delta_time = 0;
    double rad_to_deg = 57.29578;
    double last_time = 0;


public:

    bool rotation(int direction_, char dir_rotation = 'O',
        int  degrees_from_act_to_target_pos = 0) {
        /// <summary>
        /// 
        /// </summary>
        /// <param name="direction"> 0-360</param>
        /// <param name="dir_rotation"> L or R </param>
        /// <param name="degrees_from_act_to_target_pos"> difference from cur deg to wished deg</param>
        /// interval_delta_time = 0.4   interval of calculating speed and everything
        /// <returns></returns>
        //vector<int> separated_degrees_if_more_than_90;
        int separated_degrees_if_more_than_90[3] = { 0,0,0 };
        ROBOT_X_ROTATION = 0;
        ROBOT_Y_ROTATION = 0;
        ROBOT_THETA_ROTATION = 0;
        ekf_step.restart_calculation();
        while (true) {
            if ((ROBOT_THETA * rad_to_deg) == direction_) {
                return true;
            }
            else {
                //// step 30 degrees [90,90,3]
                int step_ = 90;
                int delimo = degrees_from_act_to_target_pos / step_;
                if ((degrees_from_act_to_target_pos % step_) == 0) {
                    for (int i = 0; i <= delimo; i++) {
                        //separated_degrees_if_more_than_90.push_back(step);
                        separated_degrees_if_more_than_90[i] = step_;
                    }
                }

                else {
                    for (int i = 0; i <= delimo; i++) {
                        separated_degrees_if_more_than_90[i] = step_;
                        int rest = degrees_from_act_to_target_pos % step_;
                        separated_degrees_if_more_than_90[i + 1] = rest;
                    }

                }
                //////
                for (auto angle : separated_degrees_if_more_than_90) {
                    ROBOT_X_ROTATION = 0;
                    ROBOT_Y_ROTATION = 0;
                    ROBOT_THETA_ROTATION = 0;
                    ekf_step.restart_calculation();
                    if (angle == 0) {
                        continue;
                    }
                    while (true)  {
                        curTime = millis();
                        if (curTime - prevTime >= intervalTime) {
                            ///////// SYNCHRONISATION !!!!! /////////////////
                            info_from_jetson();
                            speed_calculation(); //1 speed calculation
                            calculation_traveled_distance(); // 2 distance calculation
                            batery_measuring(); // 2.1 battery update
                            odo_rot.calculation_position_rotation(rpm_L, rpm_R, speed_time_factor, ROBOT_X_ROTATION, ROBOT_Y_ROTATION, ROBOT_THETA_ROTATION); //rotation             
                            ekf_step.calculation_step(odo_rot.position_rotation, speed_time_factor, rpm_L, rpm_R);
                            Serial.print("  rot theta after ekf  :  ");
                            Serial.println(ekf_step.state_estimate_k_updated_step[2]);


                            double error_range = 0.5;
                            double coefficient_slow = 1; //1.31
                            double pid_coefficient_low_angle = 30;
                            double current_angle_deg_ekf =
                                ekf_step.state_estimate_k_updated_step[2]; //new values of theta


                            if (current_angle_deg_ekf * coefficient_slow >= (angle)) {
                                //actual_theta = direction;
                                ROBOT_THETA = direction_;
                                ROBOT_X_ROTATION = 0;
                                ROBOT_Y_ROTATION = 0;
                                ROBOT_THETA_ROTATION = 0;
                                ekf_step.restart_calculation();
                                break;

                            }
                            else {
                                rotation_output(); //calculate PID values

                                if (dir_rotation == 'L') {
                                    actuator(3, PID_ROTATION_OUTPUT_L, PID_ROTATION_OUTPUT_R);
                                }
                                else if (dir_rotation == 'R') {
                                    actuator(4, PID_ROTATION_OUTPUT_L, PID_ROTATION_OUTPUT_R);
                                }
                                else {
                                    // O==stop car
                                    actuator(2, PID_ROTATION_OUTPUT_L, PID_ROTATION_OUTPUT_R);

                                }
                            }


                        }

                    }
                    
                    //run stop to motors
                    actuator(2, 0, 0);
                }

            };

            return true;

        }

    }




    void move_one_step(int direction_, int distance, char dir_rotation = 'O',
        int  degrees_from_act_to_target_pos = 0) {
        /// <summary>
        /// 
        /// </summary>
        /// <param name="direction"> 0-360 deg </param>
        /// <param name="distance"> 0-500cm</param>
        /// <param name="dir_rotation"> L or R</param>
        /// <param name="degrees_from_act_to_target_pos"> diff now wished direction</param>
        /// <returns></returns>
        /// 
        /// 
        ROBOT_ARRIVED = false;

        double new_distance_this_step = 0;
        ROBOT_X_ROTATION = 0;
        ROBOT_Y_ROTATION = 0;
        ROBOT_THETA_ROTATION = 0;
        ekf_step.restart_calculation();
        //dist_cm_L
        ROBOT_LAST_TRAVELED_DISTANCE = ROBOT_CURRENT_TRAVELED_DISTANCE;
        bool ok_ = rotation(direction_, dir_rotation, degrees_from_act_to_target_pos);
        if (ok_ == true) {
            ROBOT_LAST_TRAVELED_DISTANCE = ROBOT_CURRENT_TRAVELED_DISTANCE;
            ROBOT_X_ROTATION = 0;
            ROBOT_Y_ROTATION = 0;
            ROBOT_THETA_ROTATION = 0;
            ekf_step.restart_calculation();
            while (true) {
                if (ROBOT_LAST_TRAVELED_DISTANCE == 0 and distance == 0) {
                    new_distance_this_step = ROBOT_CURRENT_TRAVELED_DISTANCE;
                }
                else {
                    new_distance_this_step =
                        ROBOT_CURRENT_TRAVELED_DISTANCE - ROBOT_LAST_TRAVELED_DISTANCE;
                }

                if (new_distance_this_step >= distance || (distance == 0)) {
                    //arived
                    ROBOT_LAST_TRAVELED_DISTANCE = ROBOT_CURRENT_TRAVELED_DISTANCE;
                    break;
                }
                else if (new_distance_this_step < distance) {
                    output_straight_line(); //calculate PID straight line
                    //move ahead
                    actuator(1, PID_STRAIGHT_OUTPUT_L, PID_STRAIGHT_OUTPUT_R);
                }
            }



        }

        actuator(2, 0, 0); //stop the car
        ROBOT_ARRIVED = true; //return to Jetson, that command is done


    }



};



///////////////////////////////////// BODY STOP /////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////////////////////////////////
///@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@///////////////////////////
///@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@////////////////////////////
///@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@///////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////




void setup() {
    /////////////////////////////// pid ////////////////////////////////////////
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


    ROTATION_PID_L.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
    ROTATION_PID_L.SetMode(AUTOMATIC);
    ROTATION_PID_L.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI_L);

    ROTATION_PID_R.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
    ROTATION_PID_R.SetMode(AUTOMATIC);
    ROTATION_PID_R.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI_R);
    /////////////////////////////// pid ////////////////////////////////////////

    //////////////////////// I2c start //////////////////////////////////////////////
    Wire.begin(SLAVE_ADDRESS);
    //Wire.requestTo(receiveEvent,SLAVE_ADDRESS);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(sendData);
    //Wire.requestFrom(receiveEvent,SLAVE_ADDRESS);

  //////////////////////// I2c stop ////////////////////////////////////////////
    pinMode(oko, OUTPUT); //oko declare

  //////////////////// PID speed START /////////////////////////////////

    //////////////////////// PID speed STOP /////////////////////////
    ///////////////// Encoder setup start /////////////////////////////////////////
    pinMode(enc_L, INPUT_PULLUP);
    pinMode(enc_L_dir, INPUT);
    attachInterrupt(digitalPinToInterrupt(enc_L), counterL, RISING);

    pinMode(enc_R, INPUT_PULLUP);
    pinMode(enc_R_dir, INPUT);
    attachInterrupt(digitalPinToInterrupt(enc_R), counterR, RISING);
    ////////////////// Encoder setup stop //////////////////////////////////////////

     //////////////////////// UART START /////////////////////////////////////////////////////////
    Serial.begin(115200);  //print seiral monitor
   //////////////////////// UART STOP /////////////////////////////////////////////////////////


   ///// DC ///////////////////////
  // C D
    pinMode(enL, OUTPUT);
    pinMode(enR, OUTPUT);
    pinMode(in_1, OUTPUT);
    pinMode(in_2, OUTPUT);
    pinMode(in_3, OUTPUT);
    pinMode(in_4, OUTPUT);
    ///// DC //////////////////////////
    Serial.println(" End setup");
}


void loop() {
    ODOMETRY odom;// 3 odometry full
    KalmanOdometry ko;//kalman odometry improve the results
    ODOMETRY_ROTATION odom_rot; // odometry rotation
    KalmanStep ks; // kalman step
    BODY bod; //main class
    

    
    curTime = millis();
    if (curTime - prevTime >= intervalTime) { //400 ms or less make 100ms
        ///////// SYNCHRONISATION !!!!! /////////////////
        info_from_jetson();
        speed_calculation(); //1 speed calculation
        calculation_traveled_distance(); // 2 distance calculation
        batery_measuring(); // 2.1 battery update
        odom.calculation_position(rpm_L, rpm_R, speed_time_factor, ROBOT_X, ROBOT_Y, ROBOT_THETA); // full
        odom_rot.calculation_position_rotation(rpm_L, rpm_R, speed_time_factor, ROBOT_X_ROTATION, ROBOT_Y_ROTATION, ROBOT_THETA_ROTATION); //rotation
        Serial.print("full Theta :  ");
        Serial.println(odom.position_full[2]);
        Serial.print("  rot Theta :  ");
        Serial.println(odom_rot.position_rotation[2]);

        ko.calculation_ekf(odom.position_full, speed_time_factor, rpm_L, rpm_R);
        Serial.print("  full theta after ekf  :  ");
        Serial.println(ko.state_estimate_k_updated[2]);


        ks.calculation_step(odom_rot.position_rotation, speed_time_factor, rpm_L, rpm_R);
        Serial.print("  rot theta after ekf  :  ");
        Serial.println(ks.state_estimate_k_updated_step[2]);

        prevTime = curTime;   // update the time 

    }

    // run robot with command from Jetson

    //bod.move_one_step(TARGET_ORIENTATION_JETSON, DISTANCE_JETSON, DIRECTION, DIFFERENCE_ANGLE_JETSON);
    if (ROBOT_ARRIVED == false) {
        bod.move_one_step(0, 20); // (deg,dist,L/R,diff)
    }


    /////////////////////// DEBUG HERE ////////////////////////////////////////////// 

   /* Serial.print(" X: ");
    Serial.print(ROBOT_X);
    Serial.print("  Y: ");
    Serial.print(ROBOT_Y);
    Serial.print("  Theta deg: ");
    Serial.print(ROBOT_THETA * rad_to_deg);
    Serial.println();*/





}

////////////////////////////// END LOOP //////////////////////////////////////////////////////////////
////////////////////////////// END LOOP //////////////////////////////////////////////////////////////
////////////////////////////// END LOOP //////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////




void moving_PWM(float outPWM_L, float outPWM_R) {

    finalPWM_L = outPWM_L; //outut PIDa otiva v motor enable
    finalPWM_R = outPWM_R; //output PIDb

    if (finalPWM_L > 255) { finalPWM_L = 255; }
    if (finalPWM_L < -255) { finalPWM_L = -255; }
    if (finalPWM_R > 255) { finalPWM_R = 255; }
    if (finalPWM_R < -255) { finalPWM_R = -255; }

    if (direktion_L == DIR_AHEAD) {     //napred, mpu e + , motor L
        digitalWrite(in_1, HIGH); //backwards high motor L
        digitalWrite(in_2, LOW); //low
    }
    if (direktion_L == DIR_BACK) {    //dviji nazad mpu e -L
        digitalWrite(in_1, LOW); //forwards
        digitalWrite(in_2, HIGH);
    }
    if (direktion_R == DIR_AHEAD) {   //dviji napred motor R
        digitalWrite(in_3, HIGH);
        digitalWrite(in_4, LOW);
    }
    if (direktion_R == DIR_BACK) { // dviji R nazad
        digitalWrite(in_3, LOW);
        digitalWrite(in_4, HIGH);
    }

    finalPWM_L = abs((int)finalPWM_L) + MIN_ABSOLUTE_SPEED_L + bat_factor_pid;
    finalPWM_R = abs((int)finalPWM_R) + MIN_ABSOLUTE_SPEED_R + bat_factor_pid;

    finalPWM_L = constrain(finalPWM_L, MIN_ABSOLUTE_SPEED_L, 255);
    finalPWM_R = constrain(finalPWM_R, MIN_ABSOLUTE_SPEED_R, 255);

    analogWrite(enL, finalPWM_L);  //obryshtane beshe tuk c d 
    analogWrite(enR, finalPWM_R);
}



void counterL() {
    int valL = digitalRead(enc_L_dir); // BACK= 0 ,AHEAD= 
    if (valL == LOW) {
        directionsL = false;  dir_speed_L = false;// backward
    }
    if (valL == HIGH) {
        directionsL = true;  dir_speed_L = true; // forward
    }

    if (direktion_L == DIR_AHEAD) { countPulsesL++; } //forward
    else if (direktion_L == DIR_BACK) { countPulsesL--; } // backward



}

void counterR() {
    int valR = digitalRead(enc_R_dir);
    if (valR == HIGH) {
        directionsR = false;  dir_speed_R = false;// backward
    }
    if (valR == LOW) {
        directionsR = true; dir_speed_R = true; // forward
    }
    if (directionsR) { countPulsesR--; }
    else { countPulsesR++; }

}


void batery_measuring() {
    //12 V
    cur_bat = millis();

    if (cur_bat - prev_bat >= BAT_MEASSURING_INTERVAL) {
        battery = analogRead(pin_battery);   //674=3.26V    6.53V/2(3.26V)  //7 Nano
        //Serial.print(battery ); //10.84=729
        BAT = (battery / 204.6) * 3.0423; //2.95 Volts

        //bat_factor_pid=map(battery_cd,700,833,30,0);
        const int low_bat_range = 700;
        const int upper_bat_range = 870;
        if (battery > 0) {
            if (battery > upper_bat_range) { battery = upper_bat_range; }
            if (battery < low_bat_range) { battery = low_bat_range; }
            //bat_factor_pid = map(battery, 760, 870, 40, 0); //12.54-11V //for motors
            bat_factor_pid = map(battery, 760, 870, 17, 0); //12.54-11V //for motors
            // BAT_percent calculation
            BAT_percent = map(battery, 670, 845, 0, 100); //10.00 - 12.54 V //battery=(V*204.6)/ 3.0423
            if (BAT < 9) { BAT_percent = 0; }
        }
        prev_bat = cur_bat;
    }
}


void receiveEvent(int howMany) {
    for (int i = 0; i < howMany; i++) {
        temp[i] = Wire.read();
        temp[i + 1] = '\0'; //add null after ea. char
    }

    for (int i = 0; i < howMany; ++i)
        temp[i] = temp[i + 1];
    receiveFlag = true;
}

void string_to_array(char msg[]) {
    char* ptr = strtok(msg, ",");
    byte i = 0;
    int counter = 0;
    while (ptr) {
        ptr = strtok(NULL, ",");
        my_array[counter] = atol(ptr);
        i++;
        counter++;
    }
}



void sendData() {
    /// send data to Jetson nano [x(double),y(double),theta(deg,double),bat(double), arrived(0/1),T]
    String outString = String(ROBOT_X) + String("|")
        + String(ROBOT_Y) + String("|") +
        String((ROBOT_THETA * rad_to_deg)) + String("|") +
        String(BAT) + String("|") +
        String(ROBOT_ARRIVED) + String('T');

    int len = outString.length() + 1;
    char ascii_num[len];
    outString.toCharArray(ascii_num, len); /* copy string to character array */

    for (int i = 0; i < len; ++i) {
        Wire.write(ascii_num[i]);
    }
}

void calculation_traveled_distance() {
    /// calcualtion the distance and store it into  dist_cm_L and dist_cm_R
    //ROBOT_CURRENT_TRAVELED_DISTANCE 
    //ROBOT_LAST_TRAVELED_DISTANCE 
    dist_cm_L = ((sum_pulses_L / leftRev) * wheel_round);
    dist_cm_R = ((sum_pulses_R / rightRev) * wheel_round);

    ROBOT_CURRENT_TRAVELED_DISTANCE = (dist_cm_L + dist_cm_R) / 2;
}

void speed_calculation() {
    rpm_L = (double)(((countPulsesL / leftRev) * 60) / (speed_time_factor)); //0.4
    rpm_R = (double)(((countPulsesR / rightRev) * 60) / (speed_time_factor));

    ang_velocity_L = rpm_L * rpm_to_radians;
    ang_velocity_deg_L = ang_velocity_L * rad_to_deg;
    ang_velocity_R = rpm_R * rpm_to_radians;
    ang_velocity_deg_R = ang_velocity_R * rad_to_deg;

    sum_pulses_L += countPulsesL; //work only forwards
    sum_pulses_R += countPulsesR;
    countPulsesL = 0;
    countPulsesR = 0;



}



void directions_funk() {
    if (direktion_L == DIR_AHEAD) { +speedRateL; +ang_velocity_L; }
    else if (direktion_L == DIR_BACK) { -speedRateL;  -ang_velocity_L; }
    if (direktion_R == DIR_AHEAD) { +speedRateR; +ang_velocity_R; }
    else if (direktion_R == DIR_BACK) { -speedRateR;  -ang_velocity_R; }
}

void info_from_jetson() {
    //read information sent from jetson nano 
    /////////////////////////////////  I2C start  //////////////////////////////////////////////
    if (receiveFlag == true) {  //[LED,distance,direction(0-360),dir(L or R), to_target]
        string_to_array(temp);
        LED_STATUS_JETSON = my_array[0]; // 0=off, 1=on
        DISTANCE_JETSON = my_array[1]; // 0-360
        TARGET_ORIENTATION_JETSON = my_array[2];
        DIFFERENCE_ANGLE_JETSON = my_array[3];
        DIRECTION = my_array[4]; //2=left, 3=right, 4=stop, 5=ahead , 6=back, 9=null theta
        mode = my_array[5]; //1

        funk = dir_jet;

        receiveFlag = false;
    }

    //////////////////////////////////// I2C stop ////////////////////////////////////////
}


void rotation_output() {
    /// <summary>
        /// use instance.PID_ROTATION_OUTPUT_L and PID_ROTATION_OUTPUT_R
        /// </summary>

    ROTATION_PID_L.Compute(); // calculate PID 
    ROTATION_PID_R.Compute(); // calculate PID 
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
        final_speed_l = SPEED_L_target - abs(theta_pid_output_l);
        final_speed_r = SPEED_R_target + abs(theta_pid_output_r);

    }

    else if (ROBOT_THETA_ROTATION > 0) {
        final_speed_l = SPEED_L_target + abs(theta_pid_output_l);
        final_speed_r = SPEED_R_target - abs(theta_pid_output_r);

    }

    STRAIGHT_SPEED_PID_L.Compute(); // calculate PID final
    STRAIGHT_SPEED_PID_R.Compute(); // 

}



