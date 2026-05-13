#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"
#include <algorithm>	// IWYU pragma: keep

bool clawToggleState = false; // variable to keep track of the claw's state (open or closed)
bool fourBarLifted = false; // variable to keep track of the four bar lift's state (lifted or not lifted)

pros::Controller Maverick(pros::E_CONTROLLER_MASTER);
pros::Controller Goose(pros::E_CONTROLLER_PARTNER);


// left motor group
pros::MotorGroup left_motor_group({-1, -10}, pros::MotorGearset::blue);
// right motor group
pros::MotorGroup right_motor_group({11, 20}, pros::MotorGearset::blue);
// four bar lift motor
pros::MotorGroup four_bar_lift({12, 13}, pros::MotorGearset::green);

// intake motor
pros::Motor intake(14, pros::MotorGearset::blue);

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
pros::Imu imu(21);
// horizontal tracking wheel encoder 1
pros::Rotation horizontal_encoder_1(19);
// horizontal tracking wheel encoder 2
pros::Rotation horizontal_encoder_2(18);
// vertical tracking wheel encoder 1
pros::Rotation vertical_encoder_1(17);
// vertical tracking wheel encoder 2
pros::Rotation vertical_encoder_2(16);
// horizontal tracking wheel 1
lemlib::TrackingWheel horizontal_tracking_wheel_1(&horizontal_encoder_1, lemlib::Omniwheel::NEW_275, -5.75);
// horizontal tracking wheel 2
lemlib::TrackingWheel horizontal_tracking_wheel_2(&horizontal_encoder_2, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel 1
lemlib::TrackingWheel vertical_tracking_wheel_1(&vertical_encoder_1, lemlib::Omniwheel::NEW_275, -2.5);
// vertical tracking wheel 2
lemlib::TrackingWheel vertical_tracking_wheel_2(&vertical_encoder_2, lemlib::Omniwheel::NEW_275, -2.5);

// odometry settings
lemlib::OdomSensors sensors(&vertical_tracking_wheel_1, // vertical tracking wheel 1
                            &vertical_tracking_wheel_2, // vertical tracking wheel 2 
                            &horizontal_tracking_wheel_1, // horizontal tracking wheel 1
                            &horizontal_tracking_wheel_2, // horizontal tracking wheel 2
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
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

// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}

void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = Maverick.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = Maverick.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.curvature(leftY, rightX);

		// four bar manual controls (Temporary)
		if (Maverick.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			four_bar_lift.move(50);
		} 
		else if (Maverick.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			four_bar_lift.move(-50);
		} 
		else {
			four_bar_lift.move(0);
			four_bar_lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		}	

		// intake manual controls 
		if (Maverick.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			intake.move(127);
		}
		else if (Maverick.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intake.move(-127);
		}
		else {
			intake.move(0);
		}

		//claw manual controls
		if (Maverick.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            claw_piston.set_value(clawToggleState);
            clawToggleState = !clawToggleState;
        }

        // delay to save resources
        pros::delay(25);

}
}