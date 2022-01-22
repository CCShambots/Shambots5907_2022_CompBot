// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.CURRENT_LIMIT;
import static frc.robot.Constants.Drivetrain.COUNTS_PER_REV_DRIVE_MOTORS;
import static frc.robot.Constants.Drivetrain.LEFT_DRIVETRAIN_FOLLOWER;
import static frc.robot.Constants.Drivetrain.LEFT_DRIVETRAIN_LEADER;
import static frc.robot.Constants.Drivetrain.PIGEON_GYRO;
import static frc.robot.Constants.Drivetrain.RIGHT_DRIVETRAIN_FOLLOWER;
import static frc.robot.Constants.Drivetrain.RIGHT_DRIVETRAIN_LEADER;
import static frc.robot.Constants.Drivetrain.WHEEL_SIZE_INCHES;
import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.util.DriveModes.Arcade;
import static frc.robot.util.DriveModes.Limelight;
import static frc.robot.util.DriveModes.Tank;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.DriveModes;
import frc.robot.util.RobotStatus;
import frc.robot.util.TankDriveModule;
import frc.robot.util.TeleopSpeeds;

public class Drivetrain extends SubsystemBase {
  //Hardware declarations
  private static final TankDriveModule leftModule = new TankDriveModule(LEFT_DRIVETRAIN_LEADER, RIGHT_DRIVETRAIN_FOLLOWER, true);
  private static final TankDriveModule rightModule = new TankDriveModule(RIGHT_DRIVETRAIN_LEADER, RIGHT_DRIVETRAIN_FOLLOWER, true);

  private static final PigeonIMU pigeonIMU = new PigeonIMU(PIGEON_GYRO);

  private static final Compressor compressor = new Compressor(Constants.Drivetrain.COMPRESSOR, PneumaticsModuleType.CTREPCM);
  private static final DoubleSolenoid shifter = new DoubleSolenoid(Constants.Drivetrain.COMPRESSOR, PneumaticsModuleType.CTREPCM, 1, 2);

  private final ArrayList<WPI_TalonFX> motors = new ArrayList<WPI_TalonFX>();

  //Teleop object that allows easy use of joysticks to motor powers
  private DriveModes driveMode = Tank;
  private DriveModes prevDriveMode = Tank;
  private boolean reversed = false;
  private int reversedMult = 1;

  //Controllers for driving with PID Cotnrol
  
  public static PIDController linearControllerLeft = new PIDController(LINEAR_P, LINEAR_I, LINEAR_D);

  public static PIDController linearController = new PIDController(LINEAR_P, LINEAR_I, LINEAR_D);

  private double normalSpeed = .6;
  private double turboSpeed = 1;
  private double speedMult = normalSpeed;

  //Autonomous objects (odometry, trajectory following, etc)
  RamseteController controller = new RamseteController();  //Default arguments 2.0 and 0.7
  DifferentialDriveOdometry odometry;


  /**
   * Initializes the drivetrain object and adds each motor to the motors list for setup.
   */
  public Drivetrain() {
    odometry = new DifferentialDriveOdometry(getGyroHeadingOdometry(), new Pose2d());

    compressor.enableDigital();
    shifter.toggle();

  }

  //General Methods

  public void setNeutralMotorBehavior(NeutralMode mode) {
    for(TalonFX motor : motors) {
      motor.setNeutralMode(mode);
    }
  }

  public void setDampening(double smoothing) {
    for(TalonFX motor : motors) {
      motor.configOpenloopRamp(1);
    }
  }

  /**
   * @return the gyro's heading relative to gravity
   */
  public double getGyroHeading() {
    return -pigeonIMU.getFusedHeading();
  }

  /**
   * Set the fused heading (gravity oriented heading) of the pigeon gyro
   */
  public void setGyroHeading(double heading) {
    pigeonIMU.setFusedHeading(heading);
  }
  
  //Teleop Methods

  //TODO: I'm much too lazy to do this now, so we'll have to do it later
  public static void arcadeDrivePID(double linearSpeed, double turnSpeed) {

  }

  public void tankDrivePID(double inputLeft, double inputRight) {
    double speedLeft = adjustJoystick(inputLeft) * MAX_LINEAR_VELOCITY;
    double speedRight = adjustJoystick(inputRight) * MAX_LINEAR_VELOCITY;

    leftModule.setTargetVelocity(speedLeft);
    leftModule.setTargetVelocity(speedRight);
  }

  /**
   * 
   * @param input Raw joystick input
   * @return the input after being sped up, slowed down, or reversed, as per the current drivetrain settings
   */
  public double adjustJoystick(double input) {return input * speedMult * reversedMult;}

  public double getLeftVelocity() {return leftModule.getVelocity();}
  public double getRightVelocity() {return rightModule.getVelocity();}

  public double getLeftMeters() {return leftModule.getEncoderMeters();}
  public double getRightMeters() {return rightModule.getEncoderMeters();}

  public double getLeftVoltage() {return leftModule.getVoltage();}
  public double getRightVoltage() {return rightModule.getVoltage();}
  

  public DriveModes getDriveMode() {
    return driveMode;
  }

  /**Set the drive mode (tank/arcade) */
  public void setDriveMode(DriveModes mode) {
    prevDriveMode = driveMode;
    driveMode = mode;
  }

  public DriveModes getPrevDriveMode() {
    return prevDriveMode;
  }

  /**Toggle between tank and arcade drive */
  public void toggleDriveMode() {
    driveMode = driveMode == Tank ? Arcade : Tank;
  }

  public boolean isToggleDriveModeAllowed() {
    return driveMode != Limelight;
  }

  /**Change whether drivetrain is reversed or not */
  public void setReversed(boolean value) {
    this.reversed = value;

    if(value) reversedMult = -1;
    else reversedMult = 1;
  }

  public boolean getReversed() {
    return reversed;
  }

  public void toggleReversed() {
    setReversed(!reversed);
  }

  /**
   * Set the speed of the robot to normal or turbo mode
   * @param speed speed to set to
   */
  public void setSpeed(TeleopSpeeds speed) {
    speedMult = speed == TeleopSpeeds.Normal ? normalSpeed : turboSpeed;
  }

  //Autonomous Methods

  /**
   * Returns the pose of the robot in meters
   * @return the pose of the bot
   */
  public Pose2d getOdometryPose() {
    return odometry.getPoseMeters();
  }

  /**
   * resets the encoders to the given position
   * @param pose pose the robot is reset to
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();

    odometry.resetPosition(pose, getGyroHeadingOdometry());
  }

  /**
   * @return the speeds of the wheel in meters per second
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      leftModule.getVelocity(), 
      rightModule.getVelocity()
    );
  }

  public void resetEncoders() {
    leftModule.resetEncoder();
    rightModule.resetEncoder();
  }

  /**
   * Function called periodically in autonomous that updates the position of the robot
   */
  public void updateOdometry() {
    Rotation2d gyroAngle = getGyroHeadingOdometry();

    odometry.update(
      gyroAngle, 
      leftModule.getEncoderMeters(), 
      rightModule.getEncoderMeters()
    );
  }
  
  private Rotation2d getGyroHeadingOdometry() {
    //TODO: Idk whether this should be negative or not (WPILib docs says it should be)
    return Rotation2d.fromDegrees(-pigeonIMU.getFusedHeading());
  }

  @Override
  public void periodic() {
    //We have no use (yet) for odometry in teleop, so it will only update in autonomous
    if(robotStatus == RobotStatus.AUTO) {
      updateOdometry();
    }

    leftModule.runControlLoop();
    rightModule.runControlLoop();
  }
}