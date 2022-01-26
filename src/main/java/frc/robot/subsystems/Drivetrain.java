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
import static frc.robot.Constants.Drivetrain.*;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.fasterxml.jackson.databind.jsontype.impl.LaissezFaireSubTypeValidator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer.RobotStatus;
import frc.robot.util.DrivetrainModuleConstants;
import frc.robot.util.TankDriveModule;
import java.util.Map;

public class Drivetrain extends SubsystemBase {
  //Hardware declarations
  private static DrivetrainModuleConstants leftConstants= new DrivetrainModuleConstants(LEFT_P, LEFT_I, LEFT_D, LEFT_KS, LEFT_KV);
  private static final TankDriveModule leftModule = new TankDriveModule(LEFT_DRIVETRAIN_LEADER, LEFT_DRIVETRAIN_FOLLOWER, true, leftConstants);
  private static DrivetrainModuleConstants rightConstants= new DrivetrainModuleConstants(RIGHT_P, RIGHT_I, RIGHT_D, RIGHT_KS, RIGHT_KV);
  private static final TankDriveModule rightModule = new TankDriveModule(RIGHT_DRIVETRAIN_LEADER, RIGHT_DRIVETRAIN_FOLLOWER, false, rightConstants);

  private static final PigeonIMU pigeonIMU = new PigeonIMU(PIGEON_GYRO);

  private static final Compressor compressor = new Compressor(Constants.Drivetrain.COMPRESSOR, PneumaticsModuleType.CTREPCM);
  private static final DoubleSolenoid shifter = new DoubleSolenoid(Constants.Drivetrain.COMPRESSOR, PneumaticsModuleType.CTREPCM, 1, 2);

  //Teleop object that allows easy use of joysticks to motor powers
  private DriveModes driveMode = DriveModes.Tank;
  private DriveModes prevDriveMode = DriveModes.Tank;
  private boolean reversed = false;
  private int reversedMult = 1;


  private ShuffleboardTab driveTab = Shuffleboard.getTab("Drive Team");
  //Controllers for driving with PID Cotnrol
  
  public static PIDController linearControllerLeft = new PIDController(RIGHT_P, RIGHT_I, RIGHT_D);

  public static PIDController linearController = new PIDController(RIGHT_P, RIGHT_I, RIGHT_D);

  public static SlewRateLimiter leftSlewRate = new SlewRateLimiter(5);
  public static SlewRateLimiter rightSlewRate = new SlewRateLimiter(5);

  private double normalSpeed = .6;
  private double turboSpeed = 1;
  private double speedMult = normalSpeed;
  private double smoothing = 1;

  //Autonomous objects (odometry, trajectory following, etc)
  RamseteController controller = new RamseteController();  //Default arguments 2.0 and 0.7
  DifferentialDriveOdometry odometry;

  private NetworkTableEntry smoothingSlider = driveTab.add("Smoothing", smoothing)
  .withWidget(BuiltInWidgets.kNumberSlider)
  .withProperties(Map.of("min", 2, "max", 15))
  .getEntry();
private NetworkTableEntry turboSpeedSlider = driveTab.add("TurboSpeed", turboSpeed)
.withWidget(BuiltInWidgets.kNumberSlider)
.withProperties(Map.of("min", 1,"max", 4))
.getEntry();
private NetworkTableEntry breakModeToggle = driveTab.add("BreakMode", false)
.withWidget(BuiltInWidgets.kToggleButton)
.getEntry();




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
    leftModule.setNeutralMode(mode);
    rightModule.setNeutralMode(mode);
  }

  /**
   * @return the gyro's heading relative to gravity
   */
  public double getGyroHeading() {
    return pigeonIMU.getFusedHeading();
  }

  /**
   * Set the fused heading (gravity oriented heading) of the pigeon gyro
   */
  public void setGyroHeading(double heading) {
    pigeonIMU.setFusedHeading(heading);
  }

  public void setDriveTrainVariables(){
      smoothing = smoothingSlider.getDouble(5);
      leftSlewRate = new SlewRateLimiter(smoothing);
      rightSlewRate = new SlewRateLimiter(smoothing);
      turboSpeed = turboSpeedSlider.getDouble(3);
      if(breakModeToggle.getBoolean(false)){
        setNeutralMotorBehavior(NeutralMode.Coast);
      }   else{
        setNeutralMotorBehavior(NeutralMode.Brake);
      }
  }

  //Teleop Methods
  //Version of tank drive for joystick inputs
  public void tankDriveJoystick(double inputLeft, double inputRight) {tankDrivePID(inputLeft, inputRight, true, true);}

  //Version of tank drive for autonomous/trajectory inputs (in m/s)
  public void tankDriveAuto(double inputLeft, double inputRight) {tankDrivePID(inputLeft, inputRight, false, false);}

  public void tankDrivePID(double inputLeft, double inputRight, boolean applyDeadZone, boolean fromJoysticks) {
  
    double speedLeft = inputLeft;
    double speedRight = inputRight;

    if(fromJoysticks) {
      speedLeft = leftSlewRate.calculate(adjustJoystick(inputLeft) * turboSpeed * -1);
      speedRight = rightSlewRate.calculate(adjustJoystick(inputRight) * turboSpeed * -1);
    }

    leftModule.setTargetVelocity(speedLeft);
    rightModule.setTargetVelocity(speedRight);
  }

  public void arcadeDriveJoysticks(double linearInput, double turnInput) {

    WheelSpeeds tankInputs = arcadeDriveIK(adjustJoystick(-linearInput), adjustJoystick(turnInput));

    tankDrivePID(tankInputs.left, tankInputs.right, false, false);
  }

  /**
   * Arcade drive inverse kinematics to get the tank equivalent of arcade inputs
   * @param xSpeed
   * @param zRotation
   * @return Wheelspeeds object (interpreted by tankDrive method as joystick inputs)
   */
  private WheelSpeeds arcadeDriveIK(double xSpeed, double zRotation) {
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);

    // Square the inputs (while preserving the sign) to increase fine control
    // while permitting full power.
    xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
    zRotation = Math.copySign(zRotation * zRotation, zRotation);

    double leftInput;
    double rightInput;

    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

    if (xSpeed >= 0.0) {
      // First quadrant, else second quadrant
      if (zRotation >= 0.0) {
        leftInput = maxInput;
        rightInput = xSpeed - zRotation;
      } else {
        leftInput = xSpeed + zRotation;
        rightInput = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (zRotation >= 0.0) {
        leftInput = xSpeed + zRotation;
        rightInput = maxInput;
      } else {
        leftInput = maxInput;
        rightInput = xSpeed - zRotation;
      }
    }

    // Normalize the wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftInput), Math.abs(rightInput));
    if (maxMagnitude > 1.0) {
      leftInput /= maxMagnitude;
      rightInput /= maxMagnitude;
    }

    return new WheelSpeeds(leftInput, rightInput);
  }

  /**
   * 
   * @param input Raw joystick input
   * @return the input after being sped up, slowed down, or reversed, as per the current drivetrain settings
   */
  public double adjustJoystick(double input) {
    //Create dead zones
    if(Math.abs(input) < 0.05) return 0;

    double output = input * Math.abs(input);

    return output * speedMult * reversedMult;
  }

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
    driveMode = driveMode == DriveModes.Tank ? DriveModes.Arcade : DriveModes.Tank;
  }

  public boolean isToggleDriveModeAllowed() {
    return driveMode != DriveModes.Limelight;
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
    return Rotation2d.fromDegrees(getGyroHeading());
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

  //TODO: Remove these after testing
  public TankDriveModule getLeftModule() {return leftModule;}

  public TankDriveModule getRightModule() {return rightModule;}

  public static enum DriveModes {
    Tank, Arcade, Limelight
  }

  public static enum TeleopSpeeds {
    Normal, Turbo
  } 
}