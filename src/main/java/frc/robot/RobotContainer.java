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
import frc.robot.commands.drivetrain.DrivingCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.commands.drivetrain.TrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
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

import static frc.robot.Constants.Controller.*;
import static frc.robot.subsystems.Drivetrain.*;

public class RobotContainer {
  ShuffleboardTab driveTab = Shuffleboard.getTab("Drive Team");

  Field2d field = new Field2d();

  private final Drivetrain drivetrain = new Drivetrain(driveTab);
  private final Intake intake = new Intake();
  private final Conveyor conveyor = new Conveyor();

  private final Joystick driverController = new Joystick(DRIVER_CONTROLLER_PORT);//makes new Driver Controller Object
  private final Joystick operatorController = new Joystick(OPERATOR_CONTROLLER_PORT);

  //TODO: Make this based on the chosen auto route (idk how but it needs to be done!)
  private Pose2d startPose = new Pose2d();

  private SelectCommand autoCommands;
  Map<Object, Command> commands = new HashMap<>();
  SendableChooser<AutoPaths> autoChooser = new SendableChooser<>();


  public RobotContainer() {
    configureButtonBindings();
    
    Map<String, Trajectory> paths = loadPaths(List.of("Example", "CSGO-1", "CSGO-2", "CSGO-3"));


    commands.put(AutoPaths.Example, new SequentialCommandGroup(
      drivetrain.setupAuto(paths.get("Example")),
      new TrajectoryCommand(drivetrain, paths.get("Example"))

    ));

    commands.put(AutoPaths.CSGO1, new SequentialCommandGroup(
      drivetrain.setupAuto(paths.get("CSGO-1")),
      new TrajectoryCommand(drivetrain, paths.get("CSGO-1"))
    ));

    commands.put(AutoPaths.CSGO2, new SequentialCommandGroup(
      drivetrain.setupAuto(paths.get("CSGO-2")),
      new TrajectoryCommand(drivetrain, paths.get("CSGO-2"))
    ));

    commands.put(AutoPaths.CSGO3, new SequentialCommandGroup(
      drivetrain.setupAuto(paths.get("CSGO-3")),
      new TrajectoryCommand(drivetrain, paths.get("CSGO-3"))
    ));


    autoCommands = new SelectCommand(commands, this::getAutoId);

    autoChooser.setDefaultOption("example", AutoPaths.Example);
    autoChooser.addOption("CSGO-1", AutoPaths.CSGO1);
    autoChooser.addOption("CSGO-2", AutoPaths.CSGO2);
    autoChooser.addOption("CSGO-3", AutoPaths.CSGO3);

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
    Example,
    CSGO1,
    CSGO2,
    CSGO3
  }

}
