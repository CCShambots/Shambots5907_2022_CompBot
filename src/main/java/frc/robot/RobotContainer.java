// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.climber.MoveClimberCommand;
import frc.robot.commands.drivetrain.DrivingCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.commands.drivetrain.TrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.commands.LimelightTracking.BasicTrackingCommand;
import frc.robot.commands.LimelightTracking.TeleopTrackingCommand;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Turret.Direction;
import frc.robot.subsystems.Climber.ClimberState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

import static frc.robot.Constants.Controller.*;
import static frc.robot.subsystems.Drivetrain.*;

public class RobotContainer {
  ShuffleboardTab driveTab = Shuffleboard.getTab("Drive Team");

  Field2d field = new Field2d();

  private final Drivetrain drivetrain = new Drivetrain(driveTab);
  private final Intake intake = new Intake();
  private final Conveyor conveyor = new Conveyor();
  private final Turret turret = new Turret(driveTab);
  private final Climber climber = new Climber();
  

  private final Joystick driverController = new Joystick(DRIVER_CONTROLLER_PORT);//makes new Driver Controller Object
  private final Joystick operatorController = new Joystick(OPERATOR_CONTROLLER_PORT);

  private BasicTrackingCommand limeLightTurretCommand = null;

  //TODO: Make this based on the chosen auto route (idk how but it needs to be done!)
  private Pose2d startPose = new Pose2d();

  private SelectCommand autoCommands;
  Map<Object, Command> commands = new HashMap<>();
  SendableChooser<AutoPaths> autoChooser = new SendableChooser<>();


  public RobotContainer() {
    configureButtonBindings();
    
    Map<String, Trajectory> paths = loadPaths(List.of("Example"));

    commands.put(AutoPaths.Example, new SequentialCommandGroup(
      new InstantCommand(() -> {
      //  System.out.println("Setting odo pose to " + paths.get("Example").sample(0).poseMeters);
        startPose = paths.get("Example").sample(0).poseMeters;
        doAutoSetup();
      //  System.out.println("Robot pose after being set " + drivetrain.getOdometryPose());
      }),
      new TrajectoryCommand(drivetrain, paths.get("Example"))

    ));


    autoCommands = new SelectCommand(commands, this::getAutoId);

    autoChooser.setDefaultOption("example", AutoPaths.Example);

    driveTab.add(autoChooser);
    driveTab.add("field", field);

    doDrivetrainSetup();
  }

  private void configureButtonBindings() {
    //TODO: Disable Intake commands when the turret is shooting

    //Drivetrain controls
    new JoystickButton(driverController, Button.kRightBumper.value)
      .whenPressed(new InstantCommand(() -> drivetrain.setSpeed(TeleopSpeeds.Turbo)))
      .whenReleased(new InstantCommand(() -> drivetrain.setSpeed(TeleopSpeeds.Normal)));

    new JoystickButton(driverController, Button.kLeftBumper.value)
      .whenPressed(new ConditionalCommand(new InstantCommand(drivetrain::toggleDriveMode), new InstantCommand(), drivetrain::isToggleDriveModeAllowed));


    //Intake controls
    
    //Runs the intake command if the robot has fewer than two balls
    new JoystickButton(operatorController, OPERATOR_3_1)
      .whenPressed(new ConditionalCommand(new IntakeCommand(intake, conveyor, () -> operatorController.getRawButton(OPERATOR_3_3)), new InstantCommand(), () -> conveyor.getNumberOfBalls() < 2));
    

    //Turret Controls
    new JoystickButton(driverController, Button.kA.value)
        .whenPressed(new InstantCommand(() -> toggleLimelightTargeting(() -> driverController.getRawButton(Button.kLeftBumper.value))));
      
      new JoystickButton(driverController, Button.kRightBumper.value).whenPressed(new InstantCommand(() -> turret.toggleSearchDirection()));
      new JoystickButton(operatorController, 5).whenPressed(new InstantCommand(() -> turret.setSearchDirection(Direction.CounterClockwise)));
      new JoystickButton(operatorController, 9).whenPressed(new InstantCommand(() -> turret.setSearchDirection(Direction.Clockwise)));
  
      new JoystickButton(driverController, Button.kY.value).whenPressed(new InstantCommand(() -> turret.setFlywheelTarget(2000))).whenReleased(new InstantCommand(() -> turret.setFlywheelTarget(0)));
  
      new JoystickButton(operatorController, 2)
      .whenPressed(new MoveClimberCommand(climber, ClimberState.Lowered));
    
    new JoystickButton(operatorController, 4)
      .whenPressed(new MoveClimberCommand(climber, ClimberState.Mid));
  }

  public void telemetry() {
    SmartDashboard.putNumber("Gyro value", drivetrain.getGyroHeading());
    SmartDashboard.putNumber("Left meters traveled", drivetrain.getLeftMeters());
    SmartDashboard.putNumber("Right meters traveled", drivetrain.getRightMeters());
    SmartDashboard.putNumber("Left voltage", drivetrain.getLeftVoltage());
    SmartDashboard.putNumber("Right voltage", drivetrain.getRightVoltage());
    SmartDashboard.putNumber("Left velocity", drivetrain.getLeftVelocity());
    SmartDashboard.putNumber("Right velocity", drivetrain.getRightVelocity());
    SmartDashboard.putNumber("Left feed forward", drivetrain.getLeftModule().getFeedForwardOutput());
    SmartDashboard.putNumber("Right feed forward", drivetrain.getRightModule().getFeedForwardOutput());
    SmartDashboard.putNumber("Left PID", drivetrain.getLeftModule().getPIDOutput());
    SmartDashboard.putNumber("Right PID", drivetrain.getRightModule().getPIDOutput());
    SmartDashboard.putData("RightPID", drivetrain.getRightModule().getPIDController());
    SmartDashboard.putData("leftPID", drivetrain.getLeftModule().getPIDController());
    SmartDashboard.putNumber("left setpoint", drivetrain.getLeftModule().getSetpoint());
    SmartDashboard.putNumber("right setpoint", drivetrain.getRightModule().getSetpoint());
    field.setRobotPose(drivetrain.getOdometryPose());
    SmartDashboard.putNumber("robot x", drivetrain.getOdometryPose().getX());
    SmartDashboard.putNumber("robot y", drivetrain.getOdometryPose().getY());
    SmartDashboard.putNumber("robot z", drivetrain.getOdometryPose().getRotation().getDegrees());
  }

  private void toggleLimelightTargeting(BooleanSupplier shootingSupplier) {
    if(limeLightTurretCommand != null) {
      endLimelightTargeting();
    } else {
      startLimelightTargeting(new TeleopTrackingCommand(turret, shootingSupplier));
    }
  }

  private void startLimelightTargeting(BasicTrackingCommand command) {
    limeLightTurretCommand = command;

    limeLightTurretCommand.schedule();
  }

  private void endLimelightTargeting() {
    limeLightTurretCommand.cancel();

    limeLightTurretCommand = null;  
  }

  public void doDrivetrainSetup() {
    //Set the default command for easy driving
    drivetrain.setDefaultCommand(new DrivingCommand(drivetrain, () -> driverController.getRawAxis(DRIVER_LEFT_JOYSTICK_X_AXIS), 
    () -> driverController.getRawAxis(DRIVER_LEFT_JOYSTICK_Y_AXIS), () -> driverController.getRawAxis(DRIVER_RIGHT_JOYSTICK_Y_AXIS)));

    //This updates variables from the dashbaord sliders
    drivetrain.setDriveTrainVariables();

    setTeleop();
  }

  public Map<String, Trajectory> loadPaths(List<String> names) {
    Map<String, Trajectory> trajectories = new HashMap<>();

    for(String n : names) {
      String trajectoryJSON = "paths/" + n + ".wpilib.json";
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectories.put(n, TrajectoryUtil.fromPathweaverJson(trajectoryPath));
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      }
      System.out.println("Loaded path: " + n);
    }

    return trajectories;
  }

  public SelectCommand getAutoCommand() {
    return autoCommands;
  }

  public AutoPaths getAutoId() {
    return autoChooser.getSelected();
  }

  public void doAutoSetup() {
    setAutonomous();
    drivetrain.resetOdometry(startPose);
  }

  public void setAutonomous() {
    Constants.Drivetrain.robotStatus = RobotStatus.AUTO;
  }

  public void setTeleop() {
    Constants.Drivetrain.robotStatus = RobotStatus.TELEOP;
  }

  public void setDisabled() {
    Constants.Drivetrain.robotStatus = RobotStatus.DISABLED;
  }

  public static enum RobotStatus {
    AUTO, TELEOP, DISABLED
  } 

  public enum AutoPaths {
    Example
  }

}
