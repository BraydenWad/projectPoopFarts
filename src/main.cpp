#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/rtos.hpp"
#include <algorithm>	// IWYU pragma: keep
#include <cmath>

int cascadeHeightBottom = 0; // height for level 0 (rotations)
int cascadeHeight1 = 0; // height for level 1 (rotations)
int cascadeHeight2 = 0; // height for level 2 (rotations)
int cascadeHeight3 = 0; // height for level 3 (rotations)
int cascadeHeight4 = 0; // height for level 4 (rotations)
int cascadeHeight5 = 0; // height for level 5 (rotations)
int armLeftPose = 0; //absolute position for arm to be in left position (rotations)
int armRightPose = 0; //absolute position for arm to be in right position (rotations)
int armDownPose = 0; //absolute position for arm to be in down position (rotations)

double cascadeKp = 0.3; // proportional gain for cascade PID
double cascadeKi = 0.0; // integral gain for cascade PID
double cascadeKd = 0.0; // derivative gain for cascade PID
double cascadeKg = 0.0; // gravity compensation gravity for cascade PID
double cascadePIDTarget = 0.0; // target position for cascade PID
volatile double cascadeTargetValue = 0.0; // target position for cascade PID

double armKp = 0.3; // proportional gain for arm PID
double armKi = 0.0; // integral gain for arm PID
double armKd = 0.0; // derivative gain for arm PID
double armKg = 0.0; // gravity compensation gravity for arm PID
double armPIDTarget = 0.0; // target position for arm PID
volatile double armTargetValue = 0.0; // target position for arm PID

bool clawToggleState = false; // variable to keep track of the claw's state (open or closed)

pros::Controller Maverick(pros::E_CONTROLLER_MASTER);
pros::Controller Goose(pros::E_CONTROLLER_PARTNER);

// left motor group
pros::MotorGroup left_motor_group({-1, -11}, pros::MotorGearset::blue);
// right motor group
pros::MotorGroup right_motor_group({10, 20}, pros::MotorGearset::blue);
// four bar lift motor group
pros::MotorGroup cascade({12, -13}, pros::MotorGearset::blue);
//arm motor group
pros::MotorGroup arm({19, -20}, pros::MotorGearset::green);
// intake motor group
pros::MotorGroup intake({14, -15}, pros::MotorGearset::green);

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
pros::Imu imu(7);
// horizontal tracking wheel encoder 1
pros::Rotation horizontal_encoder_1(1);
// horizontal tracking wheel encoder 2
pros::Rotation horizontal_encoder_2(2);
// vertical tracking wheel encoder 1
pros::Rotation vertical_encoder_1(-3);
// vertical tracking wheel encoder 2
pros::Rotation vertical_encoder_2(4);
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
        if (fabs(armError) < 50) { 
            armIntegral += armError;
        } else {
            armIntegral = 0;
        }
        
        // Calculate Derivative (Rate of Change)
        armDerivative = armError - armPrevError;
        armPrevError = armError;
        
        // Calculate the total PID output power
        double armOutputPower = (armError * armKp) + (armIntegral * armKi) + (armDerivative * armKd) + armKg;
        
        // Cap the max/min bounds to match PROS .move() limits
        if (armOutputPower > 127) armOutputPower = 127;
        if (armOutputPower < -127) armOutputPower = -127;
        
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

    armTargetValue = 3600;
    pros::Task liftTaskHandle(armPIDTask);


    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = Maverick.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = Maverick.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.curvature(leftY, rightX);

		//Cascade controls
        if (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))         {cascade.move_absolute(cascadeHeightBottom, 127);}
        else if (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))   {cascade.move_absolute(cascadeHeight1, 127);}
        else if (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_UP))      {cascade.move_absolute(cascadeHeight2, 127);}
        else if (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))    {cascade.move_absolute(cascadeHeight3, 127);}
        else if (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_L1))      {cascade.move_absolute(cascadeHeight4, 127);}
        else if (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_L2))      {cascade.move_absolute(cascadeHeight5, 127);}
        //Manual cascade controls
        // else if (Maverick.get_digital(pros::E_CONTROLLER_DIGITAL_L1))   {cascade.move_velocity(127);}
        // else if (Maverick.get_digital(pros::E_CONTROLLER_DIGITAL_L2))   {cascade.move_velocity(-127);}
        // else    {cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        //         cascade.brake();}

        //Arm controls
        if      (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_B))       {arm.move_absolute(armDownPose, 127);} //arm down position
        else if (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_A))       {arm.move_absolute(armRightPose, 127);} //arm right position
        else if (Goose.get_digital(pros::E_CONTROLLER_DIGITAL_Y))       {arm.move_absolute(armLeftPose, 127);} //arm left position
        else    {arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);}

        //Intake controls
        if (Maverick.get_digital(pros::E_CONTROLLER_DIGITAL_R1))        {intake.move_velocity(127);} //intake
        else if (Maverick.get_digital(pros::E_CONTROLLER_DIGITAL_R2))   {intake.move_velocity(-127);} //outtake
        else {intake.move_velocity(0);} //stop intake

		//claw manual controls (Toggle)
		if (Goose.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            claw_piston.set_value(clawToggleState);
            clawToggleState = !clawToggleState;
        }

        

        // delay to save resources
        pros::delay(25);

}
}
