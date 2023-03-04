#include <PID_v1.h>
#include <Wire.h>

double ROBOT_X = 0; //main vaiable
double ROBOT_Y = 0;//main vaiable
double ROBOT_THETA = 0; //radians main vaiable 

double ROBOT_X_ROTATION = 0; // 
double ROBOT_Y_ROTATION = 0; // 
double ROBOT_THETA_ROTATION = 0; // radians //main vaiable

double TARGET_ANGLE_ROTATION = 0; // main variable

double ROBOT_X_STEP = 0; //main vaiable
double ROBOT_Y_STEP = 0;//main vaiable
double ROBOT_THETA_STEP = 0; //radians main vaiable 

double ROBOT_CURRENT_TRAVELED_DISTANCE = 0;
double ROBOT_LAST_TRAVELED_DISTANCE = 0;

int ROBOT_ARRIVED = 0; // 0=False, 1=True

bool start_straight = false;

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////




const int MIN_L = 27; //30
const int MIN_R = 30; //30

const double Kp_L_theta = 28; // # 28
const double Ki_L_theta = 97; //  # 97
const double Kd_L_theta = 0.13; // # 0.15

const double Kp_R_theta = 35.6; // # 35.6
const double Ki_R_theta = 88; // # 88
const double Kd_R_theta = 1; // # 1

const double Kp_L = 2.9; // # 2.9
const double Ki_L = 0; // # 0
const double Kd_L = 0; // # 0

const double Kp_R = 3.25; // # 3.25
const double Ki_R = 0; //  # 0
const double Kd_R = 0; // # 0


const double Kp_L_rot = 2; // # 2.9
const double Ki_L_rot = 0; // # 0
const double Kd_L_rot = 0; // # 0

const double Kp_R_rot = 2; // # 2.9
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






unsigned long start = 0;




unsigned long intervalTime = 100; //100, dt=0.4 ms
double speed_time_factor = 0.1; //0.1

const int DIR_AHEAD = 1;
const int DIR_BACK = 2;
const int DIR_STOP = 0; //stop


unsigned long cur = 0;
unsigned long prev = 0;
unsigned long cur_bat = 0;
unsigned long prev_bat = 0;

int battery = 0;
double BAT = 0; //real value of the battery 2
double bat_factor_pid = 0;  //value for pid
double bat_factor_pid_L = 0;  //value for pid
double bat_factor_pid_R = 0;  //value for pid
int bat_value = 0;
double BAT_percent = 0;

int oko = 11; // dqsno oko
int pin_battery = 7; // pin input status batery 12.6V

////////////////////////////////////

double dist_cm_L = 0;
double dist_cm_R = 0;




int BAT_MEASSURING_INTERVAL = 100;  //1500

// have to store the distance, because the spead del it
unsigned long sum_pulses_L = 0;
unsigned long sum_pulses_R = 0;

int funk = 0;
int direktion_L = 0;  //
int direktion_R = 0;  //



// variables for calculating the theta angle-orientation
double speed_L_teta = 0;
double speed_R_teta = 0;
double wheel_diameter = 6.8; //cm 6.8  21.3
double radius_wheel = 3.4; //cm 0.034m  3.4 cm  68mm diameter
double wheel_round = 21.3; // obikolka na koleloto w cm
double width_of_car = 18.0;

const double rpm_to_radians = 0.10471975512;
const double rad_to_deg = 57.2957795; // 57.2957795
const double deg_to_rad = 0.0174532925; // 0.0174532925

const float Pi = 3.141592653589793;


const int N = 3;




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



double setpoint_rot_L = TARGET_ANGLE_ROTATION;
double setpoint_rot_R = TARGET_ANGLE_ROTATION;
double CURRENT_THETA_DEG = 0;

double test_speed_rotation = ROBOT_THETA_STEP * deg_to_rad;

PID ROTATION_PID_L(&test_speed_rotation, &PID_ROTATION_OUTPUT_L, &setpoint_rot_L, Kp_L_rot, Ki_L_rot, Kd_L_rot, DIRECT);
PID ROTATION_PID_R(&test_speed_rotation, &PID_ROTATION_OUTPUT_R, &setpoint_rot_R, Kp_R_rot, Ki_R_rot, Kd_R_rot, DIRECT);




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

const double leftRev = 1445.0;  // counts per revolution motor L 1447
const double rightRev = 1441.0;  // counts per revolution motor R 1441

const int leftRevInt = 1445;  // counts per revolution motor L 1447
const int rightRevInt = 1441;  // counts per revolution motor R 1441






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


bool dir_speed_L = true;
bool dir_speed_R = true;







////////////////////// Encoder stop //////////////////////////////////////////////




bool flag = false;
double finalPWM_L;
double finalPWM_R;



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


int mode = 0;
int LED_STATUS_JETSON = 0;
int DISTANCE_JETSON = 0;
int TARGET_ORIENTATION_JETSON = 0;
int DIFFERENCE_ANGLE_JETSON = 0;
int DIRECTION = 0;

char jetson[40];
//////////////////////// I2c stop //////////////////////////////////////////////

//////////// old class var ///////////////////////////////////
bool is_restarted = false;
double result_full[3] = { 0.0,0.0,0.0 };

double position_full[3]{ 0.0,0.0,0.0 }; //use this in odometry x,y,theta
double position_rotation[3]{ 0,0,0 }; //use this in roatation
double position_rotation_step[3]{ 0,0,0 }; //use this in roatation

bool is_communication_on = false;


//////////// old class var ///////////////////////////////////


void calculation_full_odometry(double vel_l, double vel_r,
    double dt, double curr_x, double curr_y, double curr_theta) {

    double vel_L = vel_l * rpm_to_radians;
    double vel_R = vel_r * rpm_to_radians;

    double omega = (double)(((vel_L - vel_R) / (width_of_car)) * radius_wheel) * dt;

    /*Serial.print("omega= ");
    Serial.print(omega);*/

    double R = 0.0;
    if (vel_L != vel_R) {
        R = width_of_car / 2.0 * ((vel_R + vel_L) / (vel_L - vel_R));
    }
    else { R = 0.0; }
    /* Serial.print(" theta: ");
     Serial.print(curr_theta *rad_to_deg);*/


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


    //calculation the matrix


    for (int i = 0; i < 3; i++) { // All array elements
        result_full[i] = 0;
        for (int j = 0; j < 3; j++) {
            result_full[i] += R_matrix[j][i] * A[i];
        }

    }


    position_full[0] = result_full[0] + B[0];
    position_full[1] = result_full[1] + B[1];
    position_full[2] = result_full[2] + B[2];

    ROBOT_X = position_full[0];
    ROBOT_Y = position_full[1];
    ROBOT_THETA = position_full[2];

    //Serial.print(ROBOT_THETA);
    /*Serial.print(" X:");
    Serial.println(ROBOT_X);*/
    //start_straight = true;




}




void calculation_position_rotation(double vel_l, double vel_r,
    double delta_time, double curr_x, double curr_y, double curr_theta) {

    vel_l *= rpm_to_radians;
    vel_r *= rpm_to_radians;
    //curr_theta = curr_theta * (3.1415 / 180);;
    double omega = (((vel_l - vel_r) / (width_of_car)) * radius_wheel) * delta_time;


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
            result[i] += R_matrix[j][i] * A[i];
        }

    }




    position_rotation[0] = result[0] + B[0];
    position_rotation[1] = result[1] + B[1];
    position_rotation[2] = result[2] + B[2];
    //Serial.print("rot theta= ");
    //Serial.println(position_rotation[2] * rad_to_deg);


    //Serial.print("position_rotation[2] : ");
    //Serial.println(position_rotation[2]);

    ROBOT_X_ROTATION = position_rotation[0];
    ROBOT_Y_ROTATION = position_rotation[1];
    ROBOT_THETA_ROTATION = position_rotation[2];



}





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


    //Serial.print("position_rotation[2] : ");
    //Serial.println(position_rotation[2]);

    ROBOT_X_STEP = position_rotation_step[0];
    ROBOT_Y_STEP = position_rotation_step[1];
    ROBOT_THETA_STEP = position_rotation_step[2];



}



void actuator(int mode_, double pwm_l = 0, double pwm_r = 0) {
    /// control the movements of the car
    /// 1=ahead, 2=stop, 3=left, 4=right
    /// movinig all motors ahead
    if (mode_ == 1) {
        direktion_L = DIR_AHEAD; direktion_R = DIR_AHEAD;
        MIN_ABSOLUTE_SPEED_L = MIN_L; MIN_ABSOLUTE_SPEED_R = MIN_R;
        moving_PWM(pwm_l, pwm_r);
    }
    // STOP 
    else if (mode_ == 2) {
        direktion_L = DIR_STOP; direktion_R = DIR_STOP;
        moving_PWM(0, 0); MIN_ABSOLUTE_SPEED_L = 0; MIN_ABSOLUTE_SPEED_R = 0;
        rpm_L = 0.0; rpm_R = 0.0;
        bat_factor_pid = 0;
        bat_factor_pid_L = 0;
        bat_factor_pid_R = 0;
        //Serial.println("stop the car");

    }

    // TURN LEFT
    else if (mode_ == 3) {
        direktion_L = DIR_BACK; direktion_R = DIR_AHEAD;
        MIN_ABSOLUTE_SPEED_L = MIN_L; MIN_ABSOLUTE_SPEED_R = MIN_R;
        moving_PWM(pwm_l, pwm_r);

    }
    // TURN RIGHT 
    else if (mode_ == 4) {
        direktion_L = DIR_AHEAD; direktion_R = DIR_BACK;
        MIN_ABSOLUTE_SPEED_L = MIN_L; MIN_ABSOLUTE_SPEED_R = MIN_R;
        moving_PWM(pwm_l, pwm_r);
    }

    else {
        direktion_L = DIR_STOP; direktion_R = DIR_STOP;
        moving_PWM(0, 0); MIN_ABSOLUTE_SPEED_L = 0; MIN_ABSOLUTE_SPEED_R = 0;
        rpm_L = 0.0; rpm_R = 0.0;

    }



    if (LED_STATUS_JETSON == 0) { digitalWrite(oko, LOW); } // Lights on
    else if (LED_STATUS_JETSON == 1) { digitalWrite(oko, HIGH); } // lights off



}




void rotation(int direction_, int dir_rotation = 0,
    int  degrees_from_act_to_target_pos = 0) {
    int separated_degrees_if_more_than_90[3] = { 0,-1,-1 };
    int step_ = 90;
    ROBOT_X_STEP = 0.;
    ROBOT_Y_STEP = 0.;
    ROBOT_THETA_STEP = 0.;
    /*Serial.print(" direction: ");
    Serial.print(direction_);

    Serial.print(" diff: ");
    Serial.print(degrees_from_act_to_target_pos);
    Serial.print(" THETA: ");
    Serial.print(ROBOT_THETA * rad_to_deg);
    Serial.print(" d: ");
    Serial.println(dir_rotation);*/
    while (true) {
        if ((ROBOT_THETA * rad_to_deg) == direction_) {
            //return true;
            break;
        }
        else {
            if (degrees_from_act_to_target_pos >= step_) {
                //// step 90 degrees [90,90,3]                   
                int delimo = degrees_from_act_to_target_pos / step_;
                if ((degrees_from_act_to_target_pos % step_) == 0) {
                    //Serial.println("vytre ==0");
                    for (int i = 0; i < delimo; i++) {
                        separated_degrees_if_more_than_90[i] = (double)step_;
                    }
                }

                else { // need change for 180 deg
                   // Serial.println("vytre !=0");
                    for (int i = 0; i < delimo; i++) {
                        separated_degrees_if_more_than_90[i] = step_;
                    }
                    int rest = degrees_from_act_to_target_pos % step_;
                    separated_degrees_if_more_than_90[delimo] = rest;
                }

            }
            else if (degrees_from_act_to_target_pos < step_) {
                separated_degrees_if_more_than_90[0] = degrees_from_act_to_target_pos;
            }

            for (double angle : separated_degrees_if_more_than_90) {
                //Serial.print("target angle:  ");
                //Serial.println(angle);

                ROBOT_X_STEP = 0;
                ROBOT_Y_STEP = 0;
                ROBOT_THETA_STEP = 0;
                if (angle < 0) {
                    continue;
                }
                while (true) {
                    //Serial.println(" rotation  ");
                    curTime = millis();
                    if (curTime - prevTime >= intervalTime) {
                        speed_calculation(); //1 speed calculation                        
                        batery_measuring(); // 2.1 battery update
                        calculation_position_step(rpm_L, rpm_R, speed_time_factor, ROBOT_X_STEP, ROBOT_Y_STEP, ROBOT_THETA_STEP); //rotation                                      
                        prevTime = curTime;

                    }
                    double current_angle_deg_ekf = ROBOT_THETA_STEP * rad_to_deg; //degrees
                    /*Serial.print(" direction: ");
                    Serial.print(dir_rotation);

                    Serial.print(" angle turning: ");
                    Serial.print(abs(current_angle_deg_ekf));
                    Serial.print(" angle target: ");
                    Serial.println(angle);*/


                    if (abs(current_angle_deg_ekf) >= (angle)) {
                        ROBOT_THETA = direction_ * deg_to_rad;
                        /* ROBOT_X_STEP = 0;
                         ROBOT_Y_STEP = 0;
                         ROBOT_THETA_STEP = 0;*/
                        actuator(2, 0, 0);
                        break;

                    }
                    else if (abs(current_angle_deg_ekf) < (angle)) {
                        /*Serial.print(" angle turning: ");
                        Serial.print(current_angle_deg_ekf);
                        Serial.print(" angle : ");
                        Serial.println(angle);*/
                        TARGET_ANGLE_ROTATION = angle * deg_to_rad;
                        CURRENT_THETA_DEG = ROBOT_THETA_STEP;
                        rotation_output(); //calculate PID values



                        /*Serial.print("L : ");
                        Serial.print(PID_ROTATION_OUTPUT_L);
                        Serial.print("  R : ");
                        Serial.println(PID_ROTATION_OUTPUT_R);*/
                        //Serial.print("dir rotation: ");
                        //Serial.println(dir_rotation);
                        if (dir_rotation == 3) { //'L'
                            actuator(3, PID_ROTATION_OUTPUT_L + 20, PID_ROTATION_OUTPUT_R + 20);
                        }
                        else if (dir_rotation == 4) { //'R'
                            actuator(4, PID_ROTATION_OUTPUT_L + 23, PID_ROTATION_OUTPUT_R + 23);
                        }

                        else {
                            actuator(2, 0, 0);
                        }

                    }
                }

                //run stop to motors
                actuator(2, 0, 0);
                ROBOT_X_STEP = 0;
                ROBOT_Y_STEP = 0;
                ROBOT_THETA_STEP = 0;


            }
            break;


        }
    }
}



void move_one_step(double distance_) {
    //Serial.println("in move ");
    ROBOT_ARRIVED = 0;
    sum_pulses_L = 0.0; //restart distance measuring
    sum_pulses_R = 0.0;
    rpm_L = 0.0;
    rpm_R = 0.0;
    double new_distance_this_step = 0;
    ROBOT_X_ROTATION = 0;
    ROBOT_Y_ROTATION = 0;
    ROBOT_THETA_ROTATION = 0;
    ROBOT_LAST_TRAVELED_DISTANCE = ROBOT_CURRENT_TRAVELED_DISTANCE;
    while (true) {
        curTime = millis();
        if (curTime - prevTime >= intervalTime) { //400 ms or less make 100ms
            ///////// SYNCHRONISATION !!!!! /////////////////                
            speed_calculation(); //1 speed calculation
            calculation_traveled_distance(); // 2 distance calculation
            batery_measuring(); // 2.1 battery update
            calculation_full_odometry(rpm_L, rpm_R, speed_time_factor, ROBOT_X, ROBOT_Y, ROBOT_THETA); // full                               
            calculation_position_rotation(rpm_L, rpm_R, speed_time_factor, ROBOT_X_ROTATION, ROBOT_Y_ROTATION, ROBOT_THETA_ROTATION); //rotation             

            prevTime = curTime;   // update the time 

            if (ROBOT_LAST_TRAVELED_DISTANCE == 0.0 and distance_ == 0.0) {
                new_distance_this_step = ROBOT_CURRENT_TRAVELED_DISTANCE;// -ROBOT_LAST_TRAVELED_DISTANCE;
            }
            else {
                new_distance_this_step =
                    ROBOT_CURRENT_TRAVELED_DISTANCE;// -ROBOT_LAST_TRAVELED_DISTANCE;
            }
            //Serial.print("new dist: ");
            //Serial.println(new_distance_this_step);

            if (new_distance_this_step >= distance_ || (distance_ == 0.0)) {
                //arived
                ROBOT_LAST_TRAVELED_DISTANCE = ROBOT_CURRENT_TRAVELED_DISTANCE;
                actuator(2, 0, 0); //stop the car
                ROBOT_ARRIVED = 1;

                break;
            }
            else if (new_distance_this_step < distance_) {
                output_straight_line(); //calculate PID straight line  
                //Serial.print("rotation straight : ");
                //Serial.println(ROBOT_THETA_ROTATION * rad_to_deg);
                actuator(1, PID_STRAIGHT_OUTPUT_L, PID_STRAIGHT_OUTPUT_R);
            }

        }

    }

    actuator(2, 0, 0); //stop the car
    //ROBOT_ARRIVED = true; //return to Jetson, that command is done
    //update main variables


}


void move_command(int direction_, int distance, int dir_rotation = 0,
    int  degrees_from_act_to_target_pos = 0) {

    rotation(direction_, dir_rotation, degrees_from_act_to_target_pos);
    move_one_step(distance);
    ROBOT_ARRIVED = 1;


}




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

    //Wire.requestTo(receiveEvent,SLAVE_ADDRESS);
    Wire.begin(SLAVE_ADDRESS);
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
    Serial.println("@@@@@@@@@ End setup @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");

}

int counter_test = 0;
void loop() {
 
     /*if (counter_test == 0) {
         Serial.println(" right 90 deg");              
         move_command(90, 0, 4 , 90); // (deg,dist,3/4,diff) r=4,l=3
         Serial.print(" X: ");
         Serial.print(ROBOT_X);
         Serial.print("  Y: ");
         Serial.print(ROBOT_Y);
         Serial.print("  Theta deg: ");
         Serial.print(ROBOT_THETA * rad_to_deg);
         Serial.println();
         counter_test++;

     }
     else if (counter_test == 1) {
         Serial.println(" left 90 deg");
         move_command(0, 0, 3, 90); // (deg,dist,3/4,diff) r=4,l=3
         Serial.print(" X: ");
         Serial.print(ROBOT_X);
         Serial.print("  Y: ");
         Serial.print(ROBOT_Y);
         Serial.print("  Theta deg: ");
         Serial.print(ROBOT_THETA * rad_to_deg);
         Serial.println();
         counter_test++;

     }*/


    info_from_jetson();
    if (mode == 1 and ROBOT_ARRIVED == 0) {
        //Serial.println(" mode 1");
        is_communication_on = true; //false
        move_command(
            TARGET_ORIENTATION_JETSON,
            DISTANCE_JETSON,
            DIRECTION,
            DIFFERENCE_ANGLE_JETSON);
        mode = 2;
        is_restarted = false;
        ROBOT_ARRIVED = 1;
    }
    else if (mode == 2) {
        //Serial.println(" mode 2");
        is_communication_on = true;
        //info_from_jetson(); // mode 0=nothing, 1= working mode, 2 = neutral 3=restart 
    }


    else if (mode == 3 and is_restarted == false) {
        //Serial.println(" mode 3 restart");       
        ROBOT_ARRIVED = 0;
        is_restarted = true;
        mode = 0;
        is_communication_on = true;

    }
    else {
        //Serial.println(" mode 0");
        is_communication_on = true;
        //info_from_jetson(); // mode 0=nothing, 1= working mode, 2 = neutral 3=restart        
    }
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

    finalPWM_L = abs((int)finalPWM_L) + MIN_ABSOLUTE_SPEED_L + bat_factor_pid_L;
    finalPWM_R = abs((int)finalPWM_R) + MIN_ABSOLUTE_SPEED_R + bat_factor_pid_R;

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

    if (directionsL) { countPulsesL--; }
    else { countPulsesL++; }

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
            bat_factor_pid_L = map(battery, 670, 845, 17, 0); //12.54-11V //for motors
            bat_factor_pid_R = map(battery, 670, 845, 23, 0); //12.54-11V //for motors
            // BAT_percent calculation
            BAT_percent = map(battery, 670, 845, 0, 100); //10.00 - 12.54 V //battery=(V*204.6)/ 3.0423
            if (BAT < 9) { BAT_percent = 0; }
        }
        prev_bat = cur_bat;
    }
}


void receiveEvent(int howMany) {

    if (is_communication_on == true) {
        for (int i = 0; i < howMany; i++) {
            temp[i] = Wire.read();
            temp[i + 1] = '\0'; //add null after ea. char
        }

        for (int i = 0; i < howMany; ++i)
            temp[i] = temp[i + 1];
        receiveFlag = true;

    }

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

    if (is_communication_on == true) {
        int len = outString.length() + 1;
        char ascii_num[len];
        outString.toCharArray(ascii_num, len); /* copy string to character array */

        for (int i = 0; i < len; ++i) {
            Wire.write(ascii_num[i]);
        }

    }


}

void calculation_traveled_distance() {
    /// calcualtion the distance and store it into  dist_cm_L and dist_cm_R
    //ROBOT_CURRENT_TRAVELED_DISTANCE 
    //ROBOT_LAST_TRAVELED_DISTANCE 
    dist_cm_L = (double)((sum_pulses_L / leftRev) * wheel_round);
    dist_cm_R = (double)((sum_pulses_R / rightRev) * wheel_round);

    ROBOT_CURRENT_TRAVELED_DISTANCE = (dist_cm_L + dist_cm_R) / 2.0;

}

void speed_calculation() {
    rpm_L = (double)(((countPulsesL / leftRev) * 60) / (speed_time_factor)); //0.4
    rpm_R = (double)(((countPulsesR / rightRev) * 60) / (speed_time_factor));

    sum_pulses_L += abs(countPulsesL); //work only forwards
    sum_pulses_R += abs(countPulsesR);

    countPulsesL = 0.0;
    countPulsesR = 0.0;
}



void info_from_jetson() {
    //read information sent from jetson nano 
    /////////////////////////////////  I2C start  //////////////////////////////////////////////
    if (receiveFlag == true) {  //[LED,distance,direction(0-360),dir(L or R), to_target]
        string_to_array(temp);
        LED_STATUS_JETSON = my_array[0]; // 0=off, 1=on
        DISTANCE_JETSON = my_array[1]; // cm 0-500 cm
        TARGET_ORIENTATION_JETSON = my_array[2]; //0-360 deg
        DIFFERENCE_ANGLE_JETSON = my_array[3]; // difference between now and target
        DIRECTION = my_array[4]; // L or R
        mode = my_array[5]; //1

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
    //Serial.print("theta output l: ");
    //Serial.println(theta_pid_output_l);

    if (ROBOT_THETA_ROTATION == 0) {
        final_speed_l = SPEED_L_target;
        final_speed_r = SPEED_R_target;

    }

    else if (ROBOT_THETA_ROTATION < 0) {
        final_speed_l = SPEED_L_target + abs(theta_pid_output_l);
        final_speed_r = SPEED_R_target - abs(theta_pid_output_r);

    }

    else if (ROBOT_THETA_ROTATION > 0) {
        final_speed_l = SPEED_L_target - abs(theta_pid_output_l);
        final_speed_r = SPEED_R_target + abs(theta_pid_output_r);

    }

    STRAIGHT_SPEED_PID_L.Compute(); // calculate PID final
    STRAIGHT_SPEED_PID_R.Compute(); // 

}





