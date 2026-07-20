#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <algorithm>	// IWYU pragma: keep
#include <cmath>

int cascadeHeightBottom = 0; // height for level 0 (degrees)
int cascadeHeight1 = 360; // height for level 1 (degrees)
int cascadeHeight2 = 720; // height for level 2 (degrees)
int cascadeHeight3 = 1080; // height for level 3 (degrees)
int cascadeHeight4 = 1440; // height for level 4 (degrees)
int cascadeHeight5 = 1800; // height for level 5 (degrees)

int armFrontPose = 90*3; //absolute position for arm to be in front position (degrees)
int armBackPose = -90*3; //absolute position for arm to be in back position (degrees)
int armDownPose = 0; //absolute position for arm to be in down position (degrees)

double cascadeKp = 0.3; // proportional gain for cascade PID
double cascadeKi = 0.0; // integral gain for cascade PID
double cascadeKd = 0.0; // derivative gain for cascade PID
double cascadeKg = 0.0; // gravity compensation gravity for cascade PID
double cascadePIDTarget = 0.0; // target position for cascade PID
volatile double cascadeTargetValue = 0.0; // target position for cascade PID

double armKp = 0.4; // proportional gain for arm PID
double armKi = 0.05; // integral gain for arm PID
double armKd = 0.0; // derivative gain for arm PID
double armKg = 17.0; // gravity compensation gravity for arm PID
double armPIDTarget = 0.0; // target position for arm PID
volatile double armTargetValue = 0.0; // target position for arm PID

bool clawToggleState = false; // variable to keep track of the claw's state (open or closed)

pros::Controller Maverick(pros::E_CONTROLLER_MASTER);
pros::Controller Goose(pros::E_CONTROLLER_PARTNER);

// left motor group
pros::MotorGroup left_motor_group({-11, -12}, pros::MotorGearset::blue);
// right motor group
pros::MotorGroup right_motor_group({19, 20}, pros::MotorGearset::blue);
// four bar lift motor group
pros::MotorGroup cascade({14, -17}, pros::MotorGearset::blue);
//arm motor group
pros::Motor arm(16, pros::MotorGearset::red);
// intake motor 
pros::Motor intake(9, pros::MotorGearset::green);

// claw piston
pros::adi::DigitalOut claw_piston('A');

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
// horizontal tracking wheel 1
lemlib::TrackingWheel horizontal_tracking_wheel_1(&horizontal_encoder_1, lemlib::Omniwheel::NEW_2, -5.75);
// horizontal tracking wheel 2
lemlib::TrackingWheel horizontal_tracking_wheel_2(&horizontal_encoder_2, lemlib::Omniwheel::NEW_2, -5.75);
// vertical tracking wheel 1
lemlib::TrackingWheel vertical_tracking_wheel_1(&vertical_encoder_1, lemlib::Omniwheel::NEW_2, 2.6);
// vertical tracking wheel 2
lemlib::TrackingWheel vertical_tracking_wheel_2(&vertical_encoder_2, lemlib::Omniwheel::NEW_2, -2.6);
// odometry settings
lemlib::OdomSensors sensors(&vertical_tracking_wheel_1, // vertical tracking wheel 1
                            nullptr, // vertical tracking wheel 2 
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2
                            &imu // inertial sensor
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
        double cascadeOutputPower = (cascadeError * cascadeKp) + (cascadeIntegral * cascadeKi) + (cascadeDerivative * cascadeKd) + cascadeKg;
        
        // Cap the max/min bounds to match PROS .move() limits
        if (cascadeOutputPower > 127) cascadeOutputPower = 127;
        if (cascadeOutputPower < -127) cascadeOutputPower = -127;
        
        // Command the motor using PROS syntax
        cascade.move(cascadeOutputPower);
        pros::delay(20);
        
    }
}

void armPIDTask(void* param) {
    // Variables to track error across loops
    double armError = 0;
    double armPrevError = 0;
    double armIntegral = 0;
    double armDerivative = 0;
    
    while (true) {
        // Get the current position (average of left and right encoders)
        double armCurrentPosition = arm.get_position();
        
        // Calculate Proportional (Current Error)
        armError = armTargetValue - armCurrentPosition;
        
        // Calculate Integral (Accumulated Error)
        // Only integrate if we are close to the target to avoid "integral windup"
        if (fabs(armError) < 85) { 
            armIntegral += armError;
        } else {
            armIntegral = 0;
        }
        
        // Calculate Derivative (Rate of Change)
        armDerivative = armError - armPrevError;
        armPrevError = armError;
        
        // DYNAMIC GRAVITY: Scales automatically. 
        // 0 at bottom (0°), Max positive at Front (90°), Max negative at Back (-90°)
        // double dynamicGravity = armKg * sin(armCurrentPosition * M_PI / 180.0);

        // Calculate the total PID output power
        double armOutputPower = (armError * armKp) + (armIntegral * armKi) + (armDerivative * armKd) /*+ dynamicGravity*/;
        
        // Cap the max/min bounds to match PROS .move() limits
        if (armOutputPower > 127) armOutputPower = 127;
        if (armOutputPower < -127) armOutputPower = -127;
        
// DIAGNOSTIC PRINT LINES (Add these)
// pros::lcd::print(4, "Target: %.1f", armTargetValue);
// pros::lcd::print(5, "Actual Pos: %.1f", armCurrentPosition);
// pros::lcd::print(6, "Power Sent: %.1f", armOutputPower);

        // Command the motor using PROS syntax
        arm.move(armOutputPower);


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
            // pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            // pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            // pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(50);
        }
    });
}

void autonomous() {
    // cascade.tare_position();
}

void opcontrol() {
    cascade.tare_position(); //HIGHLY TEMPORARY, REMOVE LATER
    arm.tare_position(); //HIGHLY TEMPORARY, REMOVE LATER
    pros::Task armTaskHandle(armPIDTask);
    pros::Task cascadeTaskHandle(cascadePIDTask);

    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = Maverick.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = Maverick.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.curvature(leftY, rightX);

		//Cascade controls
        if      (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))    {cascadeTargetValue = cascadeHeightBottom;}
        else if (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))   {cascadeTargetValue = cascadeHeight1;}
        else if (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_UP))      {cascadeTargetValue = cascadeHeight2;}
        else if (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))    {cascadeTargetValue = cascadeHeight3;}
        else if (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_L1))      {cascadeTargetValue = cascadeHeight4;}
        else if (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_L2))      {cascadeTargetValue = cascadeHeight5;}

        //Arm controls
        if      (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_B))       {armTargetValue = armDownPose;} //arm down position
        else if (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_A))       {armTargetValue = armBackPose;} //arm right position
        else if (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_Y))       {armTargetValue = armFrontPose;} //arm left position

        //Intake controls
        if      (Maverick.get_digital(pros::E_CONTROLLER_DIGITAL_R1))   {intake.move(127);} //intake
        else if (Maverick.get_digital(pros::E_CONTROLLER_DIGITAL_R2))   {intake.move(-127);} //outtake
        else    {intake.move(0);} //stop intake

		//claw manual controls (Toggle)
		if (Goose.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {claw_piston.set_value(clawToggleState);
                                                                                clawToggleState = !clawToggleState;}

        

        // delay to save resources
        pros::delay(25);
}
}
