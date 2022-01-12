// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Drivetrain stuff

    public static final int RIGHT_DRIVETRAIN_LEADER = 0;
    public static final int RIGHT_DRIVETRAIN_FOLLOWER = 1;
    public static final int LEFT_DRIVETRAIN_LEADER = 2;
    public static final int LEFT_DRIVETRAIN_FOLLOWER = 3;

    public static final int PIGEON_GYRO = 0;

    public static final SupplyCurrentLimitConfiguration CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(true, 20, 20, 0.1); //enable these limits, current limit, trigger threshold, trigger threshold time
    
    public static final double COUNTS_PER_REV_DRIVE_MOTORS = 2048;
    //TODO: Get wheel size
    public static final double WHEEL_SIZE_INCHES = 0;

    //Controller stuff
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int DRIVER_LEFT_JOYSTICK_Y_AXIS = 1; //Left wheel control
    public static final int DRIVER_RIGHT_JOYSTICK_Y_AXIS = 5; //Right wheel control
    public static final int DRIVER_BUTTON_6 = 6; //Turbo/normal speed control
}