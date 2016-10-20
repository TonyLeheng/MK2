/*! ******************************************************************************
   \file uarm_library.h
   \brief uarm library header
   \author Joey Song
   \update Joey Song, Alex Tan, Dave Corboy
   \date 12/Dec/2014
   \License GNU
   \Copyright 2016 UFactory Team. All right reserved
* *******************************************************************************/
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include "UFServo.h"
#include "linreg.h"

#ifndef uArm_library_h
#define uArm_library_h

//for the sys status
#define NORMAL_MODE                 0
#define NORMAL_BT_CONNECTED_MODE    1
#define LEARNING_MODE               2
#define SINGLE_PLAY_MODE            3
#define LOOP_PLAY_MODE              4

#define LEARNING_MODE_STOP          5 //for the record() function to stop recording
//for the different version of hardware
 #define LATEST_HARDWARE

// for the external eeprom
#ifdef LATEST_HARDWARE
   #define EXTERNAL_EEPROM_SYS_ADDRESS 0xA2
   #define EXTERNAL_EEPROM_USER_ADDRESS 0xA0
#else
   #define EXTERNAL_EEPROM_SYS_ADDRESS  0xA0
#endif
#define DATA_LENGTH  0x20
#define LEFT_SERVO_ADDRESS   0x0000
#define RIGHT_SERVO_ADDRESS  0x02D0
#define ROT_SERVO_ADDRESS    0x05A0

#ifdef LATEST_HARDWARE
	#define current_ver         "[SH3-S2.0.1]"
#else
   #define current_ver         "[SH2-S2.0.1]"
#endif

#define SERVO_ROT_NUM           0
#define SERVO_LEFT_NUM          1
#define SERVO_RIGHT_NUM         2
#define SERVO_HAND_ROT_NUM      3

#define SERVO_ROT_PIN           11
#define SERVO_LEFT_PIN          13
#define SERVO_RIGHT_PIN         12
#define SERVO_HAND_ROT_PIN      10

#define SERVO_ROT_ANALOG_PIN 2
#define SERVO_LEFT_ANALOG_PIN 0
#define SERVO_RIGHT_ANALOG_PIN 1
#define SERVO_HAND_ROT_ANALOG_PIN 3

// Old Control method Stretch / Height

/*#define ARM_STRETCH_MIN   0
#define ARM_STRETCH_MAX   300
#define ARM_HEIGHT_MIN    -100
#define ARM_HEIGHT_MAX    250*/
#define LOWER_ARM_MAX_ANGLE    145 //must be modified for different version
#define LOWER_ARM_MIN_ANGLE    1//must be modified for different version
#define UPPER_ARM_MAX_ANGLE    100//must be modified for different version
#define UPPER_ARM_MIN_ANGLE    1//must be modified for different version
#define LOWER_UPPER_MAX_ANGLE  150//must be modified for different version
#define LOWER_UPPER_MIN_ANGLE  15//must be modified for different version

#define LIMIT_SW                2    // LIMIT Switch Button
#define BUZZER                  3    // HIGH = ON
#define BTN_D4                  4    // LOW = Pressed
#define BTN_D7                  7    // LOW = Pressed

#ifdef LATEST_HARDWARE
    #define POW_DET                 A6   // power detection
    #define PUMP_GRI_STATUS         A7   // pump status feedback
    #define PUMP_GRI_EN             5    // HIGH = Pump ON
    #define SYS_LED                 6
    #define BT_DETEC                3    //share the port with buzzer
#else
    #define PUMP_EN                 6    // HIGH = Valve OPEN
    #define VALVE_EN                5    // HIGH = Pump ON
    #define GRIPPER                 9    // LOW = Catch
    #define GRIPPER_FEEDBACK        A6
#endif

#define MATH_PI 3.141592653589793238463
#define MATH_TRANS  57.2958
#define MATH_L1 90.00
#define MATH_L2 21.17
#define MATH_LOWER_ARM 148.25
#define MATH_UPPER_ARM 160.2
#define MATH_FRONT_HEADER 25.00// the distance between wrist to the front point we use
#define MATH_UPPER_LOWER MATH_UPPER_ARM/MATH_LOWER_ARM
//for the move_to function
#define IN_RANGE                  1
#define OUT_OF_RANGE_NO_SOLUTION  2
#define OUT_OF_RANGE              3

#define RELATIVE 1
#define ABSOLUTE 0
//for the pump gripper function
#define GRABBING        0
#define WORKING         1
#define STOP            2
#define PUMP_GRABBING_CURRENT 55
//ADC value of the front 9g servo
#define SERVO_9G_MAX    460
#define SERVO_9G_MIN    98
// movement path types
#define PATH_LINEAR     0   // path based on linear interpolation
#define PATH_ANGLES     1   // path based on interpolation of servo angles

// movement absolute/relative flags
#define ABSOLUTE  0
#define RELATIVE  1
//#define F_HAND_RELATIVE 2   // standard relative, current + hand parameter
//#define F_HAND_ROT_REL  4   // hand keeps orientation relative to rotationxn servo (+/- hand parameter)


// interpolation types
#define INTERP_EASE_INOUT_CUBIC 0  // original cubic ease in/out
#define INTERP_LINEAR           1
#define INTERP_EASE_INOUT       2  // quadratic easing methods
#define INTERP_EASE_IN          3
#define INTERP_EASE_OUT         4

#define LINEAR_INTERCEPT        1
#define LINEAR_SLOPE            2

//uart feedback
#define SS   "[S]"
#define S0  "[S0]"
#define S1  "[S1]"
#define S2  "[S2]"
#define FF   "[F]"
#define F0  "[F0]"
#define F1  "[F1]"

//getValue() function return
#define OK                      0
#define ERR1                    1
#define ERR2                    2  

class uArmClass
{
public:
    uArmClass();
    void arm_setup();
    double read_servo_offset(byte servo_num);
    void read_servo_calibration_data(double *rot, double *left, double *right);
    void detach_servo(byte servo_num);
    void alert(byte times, byte runt_time, byte stop_time);
    void detach_all_servos();
    void write_servo_angle(byte servo_num, double servo_angle);
    double analog_to_angle(int input_angle, byte servo_num);

    void arm_process_commands();
    bool available();

    unsigned char move_to(double x, double y, double z, double hand_angle, byte relative_flags, double times, byte ease_type, boolean enable_hand, bool polar);
    unsigned char move_to(double x, double y,double z, bool polar) {
        return move_to(x, y, z, 0, ABSOLUTE, 1.0, INTERP_EASE_INOUT_CUBIC, false, polar);
        //return move_to(x, y, z, 0, ABSOLUTE, 1.0, INTERP_EASE_INOUT_CUBIC, false);
    }
    unsigned char move_to(double x, double y,double z,double times, bool polar) {
        return move_to(x, y, z, 0, ABSOLUTE, times, INTERP_EASE_INOUT_CUBIC, true, polar);
    }

    unsigned char get_current_xyz(double *cur_rot, double *cur_left, double *cur_right, double *g_current_x, double *g_current_y, double *g_current_z, bool for_movement );
    void get_current_rotleftright();
    void calibration_data_to_servo_angle(double *data,unsigned int address);
    void read_servo_angle(byte servo_number, bool original_data);
    void read_servo_angle(byte servo_number)
    {
        read_servo_angle(servo_number, false);
    }

    unsigned char coordinate_to_angle(double x, double y, double z, double *theta_1, double *theta_2, double *theta_3, bool data_constrain);
	unsigned char limit_range(double *rot, double *left, double *right, bool data_constrain);
    void interpolate(double start_val, double end_val, double *interp_vals, byte ease_type);

    void gripper_catch(bool value);
    unsigned char gripper_status();
    void pump_catch(bool value);
    unsigned char pump_status();

    Servo g_servo_rot;
    Servo g_servo_left;
    Servo g_servo_right;
    Servo g_servo_hand_rot;

    int write_servos_angle(byte servo_rot_angle, byte servo_left_angle, byte servo_right_angle, byte servo_hand_rot_angle, byte trigger);
    int write_servos_angle(double servo_rot_angle, double servo_left_angle, double servo_right_angle, double servo_hand_rot_angle);
    int write_servos_angle(double servo_rot_angle, double servo_left_angle, double servo_right_angle);
    void attach_all();
    void attach_servo(byte servo_num);
    void runCommand(String cmnd);
    void printf(bool success, double *dat, char *letters, unsigned char num);
    void printf(bool success, int dat);
    void printf(bool success, double dat);
	char getValue(char *cmnd,const char *parameters, int parameterCount, double *valueArray);
private:
    void delay_us();
    void iic_start();
    void iic_stop();
    unsigned char read_ack();
    void send_ack();
    void iic_sendbyte(unsigned char dat);
    unsigned char iic_receivebyte();
    unsigned char iic_writebuf(unsigned char *buf,unsigned char device_addr,unsigned int addr,unsigned char len);
    unsigned char iic_readbuf(unsigned char *buf,unsigned char device_addr,unsigned int addr,unsigned char len);

    bool record();
    bool play();
    void recording_write(unsigned int address, unsigned char * data_array, int num);
    void recording_read(unsigned int address, unsigned char * data_array, int num);
protected:
    double cur_rot = 90;
    double cur_left = 90;
    double cur_right = 90;
    double cur_hand = 90;

    double g_current_x = 0;
    double g_current_y = 200;
    double g_current_z = 100;

    boolean move_to_the_closest_point = false;

    unsigned int INTERP_INTVLS;

    unsigned char move_times = 255;//255 means no move
    unsigned long buzzerStopTime=0;
    unsigned long moveStartTime=0;
    unsigned int microMoveTime=0;
    // the arrays to store the xyz coordinates first and then change to the rot left right angles
    double x_array[60];
    double y_array[60];
    double z_array[60];
    double hand_speed=10;//to save the memory

    //offset of assembling

    float LEFT_SERVO_OFFSET  =   -21;//3.8ALEX//1mine//18.6liebao   //1.3Degree
    float RIGHT_SERVO_OFFSET =   8.4;//12.5ALEX//5.6mine//-1liebao   //5.8Degree
    float ROT_SERVO_OFFSET   =   0;//7ALEX//0mine//-7liebao

    //sys status
    unsigned char sys_status = NORMAL_MODE;
    unsigned char time_50ms = 0;//used to change the led blink time
    unsigned char time_ticks = 0;

    //learning mode
    unsigned int addr = 0;

};

extern uArmClass uarm;

#endif
