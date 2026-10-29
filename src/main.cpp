#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "liblvgl/core/lv_obj_class.h" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"   // IWYU pragma: keep
#include "pros/adi.hpp"   // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <algorithm>	// IWYU pragma: keep
#include <cmath>

//Cascade heights
double cascadeBottomHeight = 0; // bottom height for cascade
double cascadeHeight1 = 200; // low height for cascade
double cascadeHeight2 = 400; // mid height for cascade
double cascadeHeight3 = 600; // high height for cascade
double cascadeHeight4 = 800; // max height for cascade

//Cascade values
double cascadeMaximumPosition = 1000; // maximum position for cascade
double cascadeKp = 0.9; // proportional gain for cascade PID
double cascadeKi = 0.0; // integral gain for cascade PID
double cascadeKd = 0.0; // derivative gain for cascade PID
double cascadePIDTarget = 0.0; // target position for cascade PID
volatile double cascadeTargetValue = 0.0; // target position for cascade PID

//Wrist positions
double wristDeployed = 360;
double wristStowed = 0;

//Wrist values
double wristKp = 1.0; // proportional gain for wrist PID
double wristKi = 0.0; // integral gain for wrist PID
double wristKd = 0.0; // derivative gain for wrist PID
double wristPIDTarget = 0.0; // target position for wrist PID
volatile double wristTargetValue = 0.0; // target position for wrist PID


//Intake values
int intakeSpeed = 127; // speed for intake motors

//End effector values
int endEffectorSpeed = 127; // speed for end effector motors

//Wrist values
int wristSpeed = 127; // speed for wrist motor

//Cascade values
int cascadeSpeed = 127; // speed for cascade motors

//Controllers
pros::Controller Maverick(pros::E_CONTROLLER_MASTER);
pros::Controller Goose(pros::E_CONTROLLER_PARTNER);

// left motor group
pros::MotorGroup left_motor_group({-15, -12}, pros::MotorGearset::blue);
// right motor group
pros::MotorGroup right_motor_group({18, 20}, pros::MotorGearset::blue);
// four bar lift motor group
pros::MotorGroup cascade({14, -17}, pros::MotorGearset::red);

//arm motor
pros::Motor endEffector(11, pros::MotorGearset::green);

//intake motor
pros::Motor intake(7, pros::MotorGearset::green);

// wrist motor
pros::Motor wrist(19, pros::MotorGearset::green); // Port 19, green gearset

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
pros::Rotation horizontal_encoder_1(14);
// horizontal tracking wheel encoder 2
// pros::Rotation horizontal_encoder_2(3);
// vertical tracking wheel encoder 1
pros::Rotation vertical_encoder_1(-15);
// vertical tracking wheel encoder 2
// pros::Rotation vertical_encoder_2(5);
//optical sensor
pros::Optical optical(7);
// horizontal tracking wheel 1
lemlib::TrackingWheel horizontal_tracking_wheel_1(&horizontal_encoder_1, lemlib::Omniwheel::NEW_2, -5.75);
// horizontal tracking wheel 2
// lemlib::TrackingWheel horizontal_tracking_wheel_2(&horizontal_encoder_2, lemlib::Omniwheel::NEW_2, -5.75);
// vertical tracking wheel 1
lemlib::TrackingWheel vertical_tracking_wheel_1(&vertical_encoder_1, lemlib::Omniwheel::NEW_2, 2.6);
// vertical tracking wheel 2
// lemlib::TrackingWheel vertical_tracking_wheel_2(&vertical_encoder_2, lemlib::Omniwheel::NEW_2, -2.6);
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

void wristPIDTask(void* param) {
    // Variables to track error across loops
    double wristError = 0;
    double wristPrevError = 0;
    double wristIntegral = 0;
    double wristDerivative = 0;
    
    while (true) {
        // Get the current position (average of left and right encoders)
        double wristCurrentPosition = wrist.get_position();
        
        // Calculate Proportional (Current Error)
        wristError = wristPIDTarget - wristCurrentPosition;
        
        // Calculate Integral (Accumulated Error)
        // Only integrate if we are close to the target to avoid "integral windup"
        if (fabs(wristError) < 50) { 
            wristIntegral += wristError;
        } else {
            wristIntegral = 0;
        }
        
        // Calculate Derivative (Rate of Change)
        wristDerivative = wristError - wristPrevError;
        wristPrevError = wristError;
        
        // Calculate the total PID output power
        double wristOutputPower = (wristError * wristKp) + (wristIntegral * wristKi) + (wristDerivative * wristKd);
        
        // Cap the max/min bounds to match PROS .move() limits
        if (wristOutputPower > 127) wristOutputPower = 127;
        if (wristOutputPower < -127) wristOutputPower = -127;
        
        // Command the motor using PROS syntax
        wrist.move(wristOutputPower);
        pros::delay(20);
        
    }
}

// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    cascade.tare_position_all(); // reset cascade motor positions
    wrist.tare_position(); // reset wrist motor position
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

void autonomous() {
    chassis.setPose(0,0,0);
    chassis.moveToPose(0, -20, 0, 2000, { .forwards = false, .minSpeed = 127});
    chassis.moveToPose(0, 10, 0, 2000, {.minSpeed = 127});
    chassis.moveToPose(0, -20, 0, 2000, { .forwards = false, .minSpeed = 127});

    // pros::Task cascadeTaskHandle(cascadePIDTask);
    // chassis.setPose(0, 0, 0); // Set robot position to (0, 0, 0)

    // //Spin roller
    // endEffector.move(endEffectorSpeed); // spin end effector
    // wrist.toggle(); // flick wrist
    // cascadeTargetValue = cascadeHeight2; // move cascade to low height
    // pros::delay(500); // wait for 0.5 seconds
    // endEffector.move(0); // stop end effector

    // //Go to score at first goal
    // chassis.moveToPose(0, 0, 0, 5000);
    // chassis.moveToPose(-5.919, 28.304, 340, 5000);
    // chassis.moveToPose(-9.724, 22.312, 34, 5000, {.forwards = false});
    // cascadeTargetValue = cascadeBottomHeight; // move cascade to high height
    // endEffector.move(-endEffectorSpeed); // spin end effector in reverse
    // pros::delay(500); // wait for 0.5 seconds
    // endEffector.move(0); // stop end effector

    // //Go to score at second goal
    // cascadeTargetValue = cascadeHeight1; // move cascade to high height
    // wrist.toggle();
    // intake.move(intakeSpeed); // start intake
    // endEffector.move(endEffectorSpeed); // spin end effector
    // chassis.moveToPose(-11.626, 32.585, 325, 5000);
    // pros::delay(500);
    // intakePivot.toggle();
    // chassis.moveToPose(-15.05, 24.12, 0, 5000, {.forwards = false});
    // intake.move(0);
    // endEffector.move(0);
    // wrist.toggle();
    // cascadeTargetValue = cascadeBottomHeight; // move cascade to high height
    // endEffector.move(-endEffectorSpeed); // spin end effector in reverse
    // pros::delay(500);
    // endEffector.move(0); // stop end effector

    // //Go to score third goal
    // chassis.moveToPoint(-21.177, 32.041, 5000);
    // chassis.turnToPoint(-38.297, 14.541, 5000);
    // intake.move(intakeSpeed); // start intake
    // endEffector.move(endEffectorSpeed); // spin end effector
    // intakePivot.toggle();
    // chassis.moveToPose(-38.297, 14.541, 225, 5000);
    // pros::delay(500);
    // intakePivot.toggle();
    // intake.move(0);
    // endEffector.move(0);







}

void opcontrol() {
    // pros::Task wristTaskHandle(wristPIDTask);

    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = Maverick.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = Maverick.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.curvature(leftY, rightX);

        //Cascade controls
        if      (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_X))   {cascade.move(cascadeSpeed);} //move end effector up
        else if (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_B))   {cascade.move(-cascadeSpeed);} //move end effector down
        else    {cascade.move(0);} //stop end effector

        //Intake
        if      (Maverick.get_digital(pros::E_CONTROLLER_DIGITAL_UP))   {endEffector.move(endEffectorSpeed);} //move end effector up
        else if (Maverick.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {endEffector.move(-endEffectorSpeed);} //move end effector down
        else    {endEffector.move(0);} //stop end effector

        //End effector controls
        if      (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_UP))   {endEffector.move(endEffectorSpeed);} //move end effector up
        else if (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {endEffector.move(-endEffectorSpeed);} //move end effector down
        else    {endEffector.move(0);} //stop end effector

        //Intake controls
        if      (Maverick.get_digital(pros::E_CONTROLLER_DIGITAL_R1))   {intake.move(intakeSpeed);}
        else if (Maverick.get_digital(pros::E_CONTROLLER_DIGITAL_R2))   {intake.move(-intakeSpeed);
                                                                                endEffector.move(endEffectorSpeed);} //outtake
        else    {intake.move(0);}

		//Wrist manual controls 
		// if (Goose.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {wristTargetValue = wristStowed;} //toggle claw piston
		// if (Goose.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {wristTargetValue = wristDeployed;} //toggle claw piston
		if      (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))  {wrist.move(30);} //toggle claw piston
		else if (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {wrist.move(-30);} //toggle claw piston
		else    {wrist.move(0);
                 wrist.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);} //stop wrist

        // delay to save resources
        pros::delay(25);
}
}
