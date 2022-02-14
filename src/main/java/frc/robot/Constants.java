// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer.RobotStatus;
import frc.robot.util.Range;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //TODO: Tune left DT ff and PID
    //TODO: Tune right DT ff and PID
    //TODO: Tune Spinner ff and PID
    //TODO: Tune bottom flywheel ff and PID
    //TODO: Tune top flywheel ff and PID
    //TODO: Tune left climber ff and PID

    //Current limit for stopping motors from exceeding max power draw
    public static final SupplyCurrentLimitConfiguration CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(true, 10, 10, 0.1); //enable these limits, current limit, trigger threshold, trigger threshold time

    public static class Drivetrain {
        //Hardware ID's
        public static final int RIGHT_DRIVETRAIN_LEADER = 01;
        public static final int RIGHT_DRIVETRAIN_FOLLOWER = 02;
        public static final int LEFT_DRIVETRAIN_LEADER = 04;
        public static final int LEFT_DRIVETRAIN_FOLLOWER = 03;

        public static final int PIGEON_GYRO = 05;
        
        public static final int COMPRESSOR_ID = 06;

        //Values for converting the motor counts to distance traveled
        public static final double COUNTS_PER_REV = 2048; //Drivetrain motors (falcon 500)
        public static final double WHEEL_SIZE = 6; //In inches

        //Robot mode (for odometry toggling)
        public static RobotStatus robotStatus = RobotStatus.AUTO;

        public static double LEFT_P = 0.05;
        public static double LEFT_I = 0;
        public static double LEFT_D = 0;

        public static double LEFT_KS = 1.0052;
        public static double LEFT_KV = 1.35;

        public static double RIGHT_P = 0.05;
        public static double RIGHT_I = 0;
        public static double RIGHT_D = 0;

        public static double RIGHT_KS = 1.0052;
        public static double RIGHT_KV = 1.35;

        //The multiplier used by default when the robot is in normal mode (instead of turbo)
        public static double NORMAL_SPEED_MULT = 0.6;

        //Max velocity (in meters per second because that's what pathweaver does)
        public static final double MAX_LINEAR_VELOCITY = 2;
        public static final double MAX_LINEAR_ACCELERATION = 2;
        //Max angular velocity (in degrees per second (because radians are cringe))
        public static final double MAX_ANGULAR_VELOCITY = Math.toDegrees(Math.PI);

        public static final double MAX_VOLTAGE = 11.5;
        
        public static final double TRACK_WIDTH = 0.64135; //meters

        //Tested to be good values by WPILib
        public static final double K_RAMSETE_B = 2.0;
        public static final double K_RAMSETE_ZETA = 0.7;

        //TODO: Set this to the actual achievable error
        public static final double PREFERRED_DISTANCE_ERROR = Units.inchesToMeters(3);
        public static final double PREFERRED_ANGLE_ERROR = Units.degreesToRadians(5);
    }

    public static class Intake {
        //Hardware
        public static final int ROLLER_1_ID = 11;
        public static final int ROLLER_2_ID = 12;
        
        public static final int SOLENOID_1_PORT_1 = 0;
        public static final int SOLENOID_1_PORT_2 = 1;
        public static final int SOLENOID_2_PORT_1 = 2;
        public static final int SOLENOID_2_PORT_2 = 3;

        //Control variables
        public static final double INTAKE_SPEED = 0.5;
    }

    public static class Conveyor {
        //Hardware
        public static final int CONVEYOR_STAGE1_ID = 21;
        public static final int CONVEYOR_STAGE2_ID = 22;
        public static final int PROX_STAGE1_ID = 0; //DIO port
        public static final int PROX_STAGE2_ID = 1; //DIO port

        //Control variables
        public static final double DEFAULT_CONVEYOR_SPEED = 0.25;
    }

    
    public static final class Turret{ 
        //Hardware devices
        public static final int FLYWHEEL1 = 31;
        public static final int FLYWHEEL2 = 32;
        public static final int TURRET_SPINNER = 33;
        public static final int HALL_EFFECT_CENTER = 2; //DIO port

        //Flywheel control
        public static final double BOTTOM_FLYWHEEL_S = 0.25;
        public static final double BOTTOM_FLYWHEEL_V = 0.0017;
        public static final double BOTTOM_FLYWHEEL_P = 0.01;
        public static final double BOTTOM_FLYWHEEL_I = 0;
        public static final double BOTTOM_FLYWHEEL_D = 0;

        public static final double TOP_FLYWHEEL_S = 0.25;
        public static final double TOP_FLYWHEEL_V = 0.0017;
        public static final double TOP_FLYWHEEL_P = 0.01;
        public static final double TOP_FLYWHEEL_I = 0;
        public static final double TOP_FLYWHEEL_D = 0;

        //TODO: Tune this value to attainable error
        public static final double FLYWHEEL_ALLOWED_ERROR = 25; //The allowed error for the flywheel setpoint (in RPM)

        //Spinner control
        public static final double COUNTS_SPINNER_ENCODER = 2048;
        public static final double TURRET_GEAR_RATIO = 10.0/140.0;

        public static final double SPINNER_CLOCKWISE_LIMIT = -180; //Clockwise turns are negative
        public static final double SPINNER_COUNTERCLOCKWISE_LIMIT = 180; //Counter-clockwise turns are psotiive
        public static final double ACCEPTABLE_ERROR = 1; //How close the turret has to get to it's setpoint before isBusy() returns false
        public static final Range INVALID_SHOOTING_RANGE = new Range(200, 300);
        public static final double SEARCH_VEL = 270; //In deg/sec: The speed the spinner will search at when it doesn't have a target
        public static final double ZERO_VEL = 45; //In deg/sec: The speed the spinner use to zero out at the start of a match
        
        public static final double SPINNER_P = 0.14;
        public static final double SPINNER_I = 0.05;
        public static final double SPINNER_D = 0;
        public static final double SPINNER_MAX_VEL = 360; //Deg/sec
        public static final double SPINNER_MAX_ACCEL = 720; //Deg/sec/sec

        public static final double SPINNER_S = 1.1762;
        public static final double SPINNER_V = 0.004;
        // public static final double SPINNER_KV = 0.065;
    
        public static final SupplyCurrentLimitConfiguration CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(true, 20, 20, 0.1); //enable these limits, current limit, trigger threshold, trigger threshold time

    }

    public static class Lidar {
        public static final int MONITOR_PIN = 0;
        public static final int TRIGGER_PIN = 0;

        public static final double LIDAR_VALUE_IN_METERS = 0;
        public static final int PROX_STAGE2_ID = 2;

        public static final double DEFAULT_CONVEYOR_SPEED = 0.25;
    }

    public static class Climber {
        //Hardware
        public static final int LEFT_CLIMBER_ID = 41;
        public static final int RIGHT_CLIMBER_ID = 42;
        //TODO: Update these to actual values
        public static final int LEFT_LIMIT_SWITCH = 6;
        public static final int RIGHT_LIMIT_SWITCH = 7;

        //TODO: fix these 
        public static final int BRAKE_1_PORT_1 = 4; //Pneumatics port
        public static final int BRAKE_1_PORT_2 = 5; //Pneumatics port
        public static final int BRAKE_2_PORT_1 = 6; //Pneumatics port
        public static final int BRAKE_2_PORT_2 = 7; //Pneumatics port

        //TODO: Tune
        public static final double LEFT_P = 0;
        public static final double LEFT_I = 0;
        public static final double LEFT_D = 0;
        public static final double LEFT_KS = 1;
        public static final double LEFT_KV = .005;

        public static final double RIGHT_P = 0;
        public static final double RIGHT_I = 0;
        public static final double RIGHT_D = 0;
        public static final double RIGHT_KS = 1;
        public static final double RIGHT_KV = .005;

        public static final double MAX_VEL = 0;
        public static final double MAX_ACCEL = 0;

        //TODO: calculate these

        public static final double MID_HEIGHT = 24; //The height (in inches) that the climber will move to for the mid level climb
        public static final double LOW_HEIGHT = 12; //Same as above (in inches) but for low goal
        public static final double LOWERED_HEIGHT = 0; //The fully lowered position of the climber (also in inches)
    }

    public static class Controller {
        //Driver Controller
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int DRIVER_LEFT_JOYSTICK_Y_AXIS = 1; //Left stick y
        public static final int DRIVER_LEFT_JOYSTICK_X_AXIS = 0; //Left stick x
        public static final int DRIVER_RIGHT_JOYSTICK_Y_AXIS = 5; //Right stick y

        //Operator Controller
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }
    
}