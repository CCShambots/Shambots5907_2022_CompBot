// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase {
  //Motor declaractions
  private static final WPI_TalonFX leftMotorLeader = new WPI_TalonFX(LEFT_DRIVETRAIN_LEADER);
  private static final WPI_TalonFX leftMotorFollower = new WPI_TalonFX(LEFT_DRIVETRAIN_FOLLOWER);
  private static final WPI_TalonFX rightMotorLeader = new WPI_TalonFX(RIGHT_DRIVETRAIN_LEADER);
  private static final WPI_TalonFX rightMotorFollower = new WPI_TalonFX(RIGHT_DRIVETRAIN_FOLLOWER);

  private static final PigeonIMU pigeonIMU = new PigeonIMU(PIGEON_GYRO);

  private final ArrayList<WPI_TalonFX> motors = new ArrayList<WPI_TalonFX>();

  //Teleop object that allows easy use of joysticks to motor powers
  //TODO: Allow arcade drive
  private final DifferentialDrive tankDrivetrain = new DifferentialDrive(leftMotorLeader, rightMotorLeader);

  private double normalSpeed = .6;
  private double turboSpeed = 1;
  private double speedMult;
  //TODO: Decide if dampening is necessary
  private double smoothing = 1;

  //Autonomous objects (odometry, trajectory following, etc)
  RamseteController controller = new RamseteController();  //Default arguments 2.0 and 0.7
  DifferentialDriveOdometry odometry;

  public Drivetrain() {
    leftMotorLeader.getSelectedSensorPosition();
    leftMotorFollower.follow(leftMotorLeader);
    rightMotorFollower.follow(rightMotorFollower);
    
    motors.add(leftMotorLeader);
    motors.add(leftMotorFollower);
    motors.add(rightMotorLeader);
    motors.add(rightMotorFollower);

    for(WPI_TalonFX motor : motors) {
      motor.configSupplyCurrentLimit(CURRENT_LIMIT);
      motor.configOpenloopRamp(smoothing);
    }
  }

  /**
   * Teleop command for running the tank drive (run periodically)
   * @param speedLeft joystick values
   * @param speedRight joystick values
   */
  public void tankDrive(double speedLeft, double speedRight) {
    tankDrivetrain.tankDrive(speedLeft * speedMult, speedRight * speedMult);
  }

  public void teleopInit() {
    for(WPI_TalonFX motor : motors) {
      motor.configSupplyCurrentLimit(CURRENT_LIMIT);
      motor.configOpenloopRamp(smoothing);

      //Assuming braking should be true
      motor.setNeutralMode(NeutralMode.Brake);
    }

    speedMult = normalSpeed;
  }

  public void autoInit() {
    //TODO: Support changing position and auto path based on
    odometry = new DifferentialDriveOdometry(getGyroHeading(), new Pose2d(0.0, 13.5, new Rotation2d()));
  }

  public void updateOdometry() {
    Rotation2d gyroAngle = getGyroHeading();

    odometry.update(gyroAngle, encoderCountsToMeters(leftMotorLeader.getSelectedSensorPosition()), encoderCountsToMeters(rightMotorLeader.getSelectedSensorPosition()));
  }

  /**
   * 
   * @param counts current encoder counts on a motor
   * @return the counts in meters
   */
  private double encoderCountsToMeters(double counts) {
    return (counts / COUNTS_PER_REV_DRIVE_MOTORS) * (WHEEL_SIZE_INCHES * Math.PI) * 0.0254;
  }

  private Rotation2d getGyroHeading() {
    //TODO: Idk whether this should be negative or not (WPILib docs says it should be)
    return Rotation2d.fromDegrees(-pigeonIMU.getFusedHeading());
  }

  public void setTurboSpeed() {
    speedMult = turboSpeed;
  }

  public void setNormalSpeed() {
    speedMult = normalSpeed;
  }

  @Override
  public void periodic() {

  }
}
