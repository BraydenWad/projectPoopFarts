#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "liblvgl/core/lv_obj_class.h" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"   // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <algorithm>	// IWYU pragma: keep
#include <cmath>

//Cascade values
double cascadeMaximumPosition = 1000; // maximum position for cascade
double cascadeKp = 0.9; // proportional gain for cascade PID
double cascadeKi = 0.0; // integral gain for cascade PID
double cascadeKd = 0.0; // derivative gain for cascade PID
double cascadePIDTarget = 0.0; // target position for cascade PID
volatile double cascadeTargetValue = 0.0; // target position for cascade PID

//Intake values
int intakeSpeed = 127; // speed for intake motors

//End effector values
int endEffectorSpeed = 127; // speed for end effector motors

//Wrist values
bool wristToggleState = false; // variable to keep track of the wrist's state (open or closed)

//Controllers
pros::Controller Maverick(pros::E_CONTROLLER_MASTER);
pros::Controller Goose(pros::E_CONTROLLER_PARTNER);

// left motor group
pros::MotorGroup left_motor_group({-11, -12}, pros::MotorGearset::blue);
// right motor group
pros::MotorGroup right_motor_group({18, 20}, pros::MotorGearset::blue);
// four bar lift motor group
pros::MotorGroup cascade({14, -17}, pros::MotorGearset::green);

//arm motor
pros::Motor endEffector(-16, pros::MotorGearset::green);

//intake motor
pros::Motor intakePrimary(10, pros::MotorGearset::green);
pros::Motor intakeSecondary(9, pros::MotorGearset::green);

// intake motor  group
pros::MotorGroup intake({9, 10}, pros::MotorGearset::green);

// wrist piston
pros::adi::Pneumatics wrist('A', false); // Port 'A', initial state false

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              14, // 14 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              366.6667, // drivetrain rpm is 366.6667
                              2 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(6);
// horizontal tracking wheel encoder 1
pros::Rotation horizontal_encoder_1(2);
// horizontal tracking wheel encoder 2
pros::Rotation horizontal_encoder_2(3);
// vertical tracking wheel encoder 1
pros::Rotation vertical_encoder_1(-4);
// vertical tracking wheel encoder 2
pros::Rotation vertical_encoder_2(5);
//optical sensor
pros::Optical optical(7);
// horizontal tracking wheel 1
lemlib::TrackingWheel horizontal_tracking_wheel_1(&horizontal_encoder_1, lemlib::Omniwheel::NEW_2, -5.75);
// horizontal tracking wheel 2
lemlib::TrackingWheel horizontal_tracking_wheel_2(&horizontal_encoder_2, lemlib::Omniwheel::NEW_2, -5.75);
// vertical tracking wheel 1
lemlib::TrackingWheel vertical_tracking_wheel_1(&vertical_encoder_1, lemlib::Omniwheel::NEW_2, 2.6);
// vertical tracking wheel 2
lemlib::TrackingWheel vertical_tracking_wheel_2(&vertical_encoder_2, lemlib::Omniwheel::NEW_2, -2.6);
// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1
                            nullptr, // vertical tracking wheel 2 
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2
                            nullptr // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(15, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              5, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2.23, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

void cascadePIDTask(void* param) {
    // Variables to track error across loops
    double cascadeError = 0;
    double cascadePrevError = 0;
    double cascadeIntegral = 0;
    double cascadeDerivative = 0;
    
    while (true) {
        // Get the current position (average of left and right encoders)
        double cascadeCurrentPosition = cascade.get_position();
        
        // Calculate Proportional (Current Error)
        cascadeError = cascadeTargetValue - cascadeCurrentPosition;
        
        // Calculate Integral (Accumulated Error)
        // Only integrate if we are close to the target to avoid "integral windup"
        if (fabs(cascadeError) < 50) { 
            cascadeIntegral += cascadeError;
        } else {
            cascadeIntegral = 0;
        }
        
        // Calculate Derivative (Rate of Change)
        cascadeDerivative = cascadeError - cascadePrevError;
        cascadePrevError = cascadeError;
        
        // Calculate the total PID output power
        double cascadeOutputPower = (cascadeError * cascadeKp) + (cascadeIntegral * cascadeKi) + (cascadeDerivative * cascadeKd);
        
        // Cap the max/min bounds to match PROS .move() limits
        if (cascadeOutputPower > 127) cascadeOutputPower = 127;
        if (cascadeOutputPower < -127) cascadeOutputPower = -127;
        
        // Command the motor using PROS syntax
        cascade.move(cascadeOutputPower);
        pros::delay(20);
        
    }
}

// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    cascade.tare_position_all(); // reset cascade motor positions
    // print position to brain screen
    pros::Task screen_task([=]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(50);
        }
    });
}

void autonomous() {}

void opcontrol() {
    pros::Task cascadeTaskHandle(cascadePIDTask);

    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = Maverick.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = Maverick.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.curvature(leftY, rightX);

        //Move the cascade with left joystick
        cascade.move(Maverick.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)); // move cascade with left joystick

        //End effector pivot
        if (Goose.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {wrist.toggle();}

        //Intake
        if      (Maverick.get_digital(pros::E_CONTROLLER_DIGITAL_UP))   {endEffector.move(endEffectorSpeed);} //move end effector up
        else if (Maverick.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {endEffector.move(-endEffectorSpeed);} //move end effector down
        else    {endEffector.move(0);} //stop end effector

        //End effector controls
        if      (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_UP))   {endEffector.move(endEffectorSpeed);} //move end effector up
        else if (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {endEffector.move(-endEffectorSpeed);} //move end effector down
        else    {endEffector.move(0);} //stop end effector

        //Intake controls
        if      (Maverick.get_digital(pros::E_CONTROLLER_DIGITAL_R1))   {intakePrimary.move(intakeSpeed);
                                                                                intakeSecondary.move(intakeSpeed);} //intake
        else if (Maverick.get_digital(pros::E_CONTROLLER_DIGITAL_R2))   {intakePrimary.move(-intakeSpeed);
                                                                                intakeSecondary.move(-intakeSpeed);} //outtake
        else    {intakePrimary.move(0);
                intakeSecondary.move(0);}//stop intake

		//claw manual controls (Toggle)
		if (Maverick.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {wrist.toggle();} //toggle claw piston

        // delay to save resources
        pros::delay(25);
}
}
