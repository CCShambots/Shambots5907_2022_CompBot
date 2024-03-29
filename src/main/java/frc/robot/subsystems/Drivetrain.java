package frc.robot.subsystems;

import static frc.robot.Constants.Drivetrain.LEFT_DRIVETRAIN_FOLLOWER;
import static frc.robot.Constants.Drivetrain.LEFT_DRIVETRAIN_LEADER;
import static frc.robot.Constants.Drivetrain.PIGEON_GYRO;
import static frc.robot.Constants.Drivetrain.RIGHT_DRIVETRAIN_FOLLOWER;
import static frc.robot.Constants.Drivetrain.RIGHT_DRIVETRAIN_LEADER;
import static frc.robot.Constants.Drivetrain.*;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.PIDandFFConstants;
import frc.robot.util.TankDriveModule;
import frc.robot.util.TankDriveModule.ControlMode;
import frc.robot.util.priorityFramework.PrioritizedSubsystem;

import java.util.Map;

public class Drivetrain extends PrioritizedSubsystem {
  //Hardware declarations
  private PIDandFFConstants autoConstants = new PIDandFFConstants(AUTO_P, AUTO_I, AUTO_D, KS, KV);
  private PIDandFFConstants teleConstants = new PIDandFFConstants(TELE_P, TELE_I, TELE_D, KS, KV);

  private TankDriveModule leftModule = new TankDriveModule(LEFT_DRIVETRAIN_LEADER, LEFT_DRIVETRAIN_FOLLOWER, false, autoConstants, teleConstants);
  private TankDriveModule rightModule = new TankDriveModule(RIGHT_DRIVETRAIN_LEADER, RIGHT_DRIVETRAIN_FOLLOWER, true, autoConstants, teleConstants);

  private PigeonIMU pigeonIMU = new PigeonIMU(PIGEON_GYRO);

  private Compressor compressor = new Compressor(COMPRESSOR_ID, PneumaticsModuleType.CTREPCM);
  private PowerDistribution pdh = new PowerDistribution(PDH_ID, ModuleType.kRev);
  

  //Drivetrain control
  private DriveModes driveMode = DriveModes.Tank;
  private DriveModes prevDriveMode = DriveModes.Tank;
  private boolean reversed = false;

  //Speed controls
  private TeleopSpeeds speedMode = TeleopSpeeds.Normal;
  private double maxSpeed = MAX_LINEAR_VELOCITY;

  private double smoothing = 8;

  //Defense flag
  private boolean defending = false;

  //Odometry object for tracking robot pose
  private DifferentialDriveOdometry odometry;

  //Value that determines whether odometry should be used or not
  private boolean useOdometry = false;

  private NetworkTableEntry smoothingSlider;
  private NetworkTableEntry speedSlider;

  /**
   * Initializes the drivetrain object and adds each motor to the motors list for setup.
   */
  public Drivetrain(ShuffleboardTab driveTab) {
    odometry = new DifferentialDriveOdometry(getGyroHeadingOdometry(), new Pose2d());

    compressor.enableDigital();

    initShuffleboard(driveTab);
  }

  /**
   * Send the driving variables to Shuffleboard
   */
  private void initShuffleboard(ShuffleboardTab driveTab){

    smoothingSlider = driveTab.add("Smoothing", smoothing)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 4, "max", 8))
      .withSize(2, 1)
      .withPosition(11, 1)
      .getEntry();

    speedSlider = driveTab.add("Speed", maxSpeed)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 1,"max", 4))
      .withSize(2, 1)
      .withPosition(11, 0)
      .getEntry();

    setDriveTrainVariables();
  }

  /**
   * Setup drivetrain variables based on the states of the shuffleboard tab
   */
  public void setDriveTrainVariables(){
    smoothing = smoothingSlider.getDouble(5);
    maxSpeed = speedSlider.getDouble(3);
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
  public void resetGyroHeading(double heading) {
    pigeonIMU.setFusedHeading(heading);
  }

  //Teleop Methods

  /**
   * 
   * @param speedLeft velocity (m/s) for the left side
   * @param speedRight velocity (m/s) for the right side
   */
  public void tankDrive(double speedLeft, double speedRight) {
    leftModule.setTargetVelocity(speedLeft);
    rightModule.setTargetVelocity(speedRight);
  }

  public double getMaxSpeed() {return maxSpeed;}
  public Drivetrain.TeleopSpeeds getSpeedMode() {return speedMode;}
  public double getSmoothing() {return smoothing;}

  public double getLeftVelocity() {return leftModule.getVelocity();}
  public double getRightVelocity() {return rightModule.getVelocity();}

  public double getLeftMeters() {return leftModule.getEncoderMeters();}
  public double getRightMeters() {return rightModule.getEncoderMeters();}

  public double getLeftVoltage() {return leftModule.getVoltage();}
  public double getRightVoltage() {return rightModule.getVoltage();}

  public void setControlLoopType(ControlMode type) {
    leftModule.setPID(type);
    rightModule.setPID(type);
  }
  

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
    driveMode = driveMode == DriveModes.Tank ? DriveModes.Curvature : DriveModes.Tank;
  }


  /**Change whether drivetrain is reversed or not */
  public void setReversed(boolean value) {this.reversed = value;}
  public boolean isReversed() {return reversed;}

  public void toggleReversed() {
    setReversed(!reversed);
  }

  public boolean isDefending() {return defending;}
  public void setDefending(boolean value) {this.defending = value;}
  public void toggleDefending() {this.defending = !defending;}

  /**
   * Set the speed of the robot to normal or turbo mode
   * @param speed speed to set to
   */
  public void setSpeed(TeleopSpeeds speed) {
    this.speedMode = speed;
  }

  //Autonomous Methods

  /**
   * Returns the pose of the robot in meters
   * Pose starts at (0,0) in the bottom left corner. Angle of the robot is from -pi to pi
   * @return the pose of the bot
   */
  public Pose2d getOdometryPose() {return odometry.getPoseMeters();}

  /**
   * @return pose of the bot in feet instead of meters
   */
  public Pose2d getOdometryPoseFeet() {
    Pose2d initialPose = getOdometryPose();
    return new Pose2d(Units.metersToFeet(initialPose.getX()), Units.metersToFeet(initialPose.getY()), initialPose.getRotation());
  }

  /**
   * resets the encoders to the given position
   * @param pose pose the robot is reset to
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();

    odometry.resetPosition(pose, getGyroHeadingOdometry());
  }

  public void setUseOdometry(boolean value) {useOdometry = value;}
  public void toggleUseOdometry() {useOdometry = !useOdometry;}
  public boolean shouldUseOdometry() {return useOdometry;}

  public void resetPID() {
    leftModule.resetPID();
    rightModule.resetPID();
  }

  /**Returns the speeds of the wheel in meters per second */
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
    updateOdometry();

    leftModule.runControlLoop();
    rightModule.runControlLoop();

    //Drivetrain telemetry
    SmartDashboard.putNumber("Gyro value", getGyroHeading());
    SmartDashboard.putNumber("Left meters traveled", getLeftMeters());
    SmartDashboard.putNumber("Right meters traveled", getRightMeters());
    SmartDashboard.putNumber("Left voltage", getLeftVoltage());
    SmartDashboard.putNumber("Right voltage", getRightVoltage());
    SmartDashboard.putNumber("Left velocity", getLeftVelocity());
    SmartDashboard.putNumber("Right velocity", getRightVelocity());
    SmartDashboard.putNumber("Left feed forward", getLeftModule().getFeedForwardOutput());
    SmartDashboard.putNumber("Right feed forward", getRightModule().getFeedForwardOutput());
    SmartDashboard.putNumber("Left PID", getLeftModule().getPIDOutput());
    SmartDashboard.putNumber("Right PID", getRightModule().getPIDOutput());
    SmartDashboard.putData("RightPID", getRightModule().getPIDController());
    SmartDashboard.putData("leftPID", getLeftModule().getPIDController());
    SmartDashboard.putNumber("left setpoint", getLeftModule().getSetpoint());
    SmartDashboard.putNumber("right setpoint", getRightModule().getSetpoint());

    // SmartDashboard.putNumber("Total power draw", pdh.getTotalCurrent());
  }

  public TankDriveModule getLeftModule() {return leftModule;}
  public TankDriveModule getRightModule() {return rightModule;}

  public static enum DriveModes {
    Tank, Curvature
  }

  public static enum TeleopSpeeds {
    Normal, Turbo, Slow
  } 
}