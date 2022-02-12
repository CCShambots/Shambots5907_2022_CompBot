// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer.RobotStatus;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //Current limit for stopping motors from exceeding max power draw
    public static final SupplyCurrentLimitConfiguration CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(true, 10, 10, 0.1); //enable these limits, current limit, trigger threshold, trigger threshold time

    public static class Drivetrain {
        public static final int RIGHT_DRIVETRAIN_LEADER = 01;
        public static final int RIGHT_DRIVETRAIN_FOLLOWER = 02;
        public static final int LEFT_DRIVETRAIN_LEADER = 04;
        public static final int LEFT_DRIVETRAIN_FOLLOWER = 03;

        public static final int PIGEON_GYRO = 05;
        
        public static final int COMPRESSOR = 06;

        //Values for converting the motor counts to distance traveled
        public static final double COUNTS_PER_REV_DRIVE_MOTORS = 2048;
        public static final double WHEEL_SIZE_INCHES = 6;
        public static final double TICKS_PER_METER = 45000;

        //Robot mode (for odometry toggling)
        public static RobotStatus robotStatus = RobotStatus.AUTO;

        public static double LEFT_P = 0.05;
        public static double LEFT_I = 0;
        public static double LEFT_D = 0;

        public static double LEFT_KS = 1.0052;
        // public static double LEFT_KV = 1.35;
        public static double LEFT_KV = 2.0;

        public static double RIGHT_P = 0.05;
        public static double RIGHT_I = 0;
        public static double RIGHT_D = 0;

        public static double RIGHT_KS = 1.0052;
        public static double RIGHT_KV = 2.0;

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

        public static final double kPDriveVel = 0;

        //TODO: Set this to the actual achievable error
        public static final double PREFERRED_DISTANCE_ERROR = Units.inchesToMeters(3);
        public static final double PREFERRED_ANGLE_ERROR = Units.degreesToRadians(5);
    }

    public static class Limelight {
        //Limelight variables
        public static final double LIMELIGHT_FIND_TARGET_SPEED = 0.25;
        public static final PIDController Z_LIMELIGHT_PID = new PIDController(0, 0, 0);
        public static final double LIMELIGHT_TOLERANCE = 5;
    }

    public static class Controller {
        //Driver Controller
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int DRIVER_LEFT_JOYSTICK_Y_AXIS = 1; //Left stick y
        public static final int DRIVER_LEFT_JOYSTICK_X_AXIS = 0; //Left stick x
        public static final int DRIVER_RIGHT_JOYSTICK_Y_AXIS = 5; //Right stick y
        public static final int DRIVER_A = 1; //Turbo/normal speed control
        public static final int DRIVER_B = 2; //Arcade/Tank drive

        //Operator Controller
        public static final int OPERATOR_CONTROLLER_PORT = 0;
        public static final int OPERATOR_BUTTON_3 = 3; //Control for switching to limelight turning 
    }
}