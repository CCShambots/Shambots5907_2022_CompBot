// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.RobotStatus;

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
    public static final SupplyCurrentLimitConfiguration CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(true, 20, 20, 0.1); //enable these limits, current limit, trigger threshold, trigger threshold time

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

        //Robot mode (for odometry toggling)
        public static RobotStatus robotStatus = RobotStatus.AUTO;

        public static double LINEAR_P = 0;
        public static double LINEAR_I = 0;
        public static double LINEAR_D = 0;

        public static double KS = 1.0052;
        public static double KV = 1.5518;

        public static PIDController turnController = new PIDController(0, 0, 0);

        //Max velocity (in meters per second because that's what pathweaver does)
        public static double MAX_LINEAR_VELOCITY = 3;
        //Max angular velocity (in degrees per second (because radians are cringe))
        public static double MAX_ANGULAR_VELOCITY = Math.toDegrees(Math.PI);
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

        //Operator Controller
        public static final int OPERATOR_CONTROLLER_PORT = 0;
        public static final int OPERATOR_A = 1; //Turbo/normal speed control
        public static final int OPERATOR_B = 2; //Arcade/Tank drive
        public static final int OPERATOR_X = 3; //Drivetrain reversed or not
        public static final int OPERATOR_BUTTON_3 = 3; //Control for switching to limelight turning 
    }
}