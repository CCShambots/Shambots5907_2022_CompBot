// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.HashMap;
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

    // commands.put(Example, new SequentialCommandGroup(
    //   new InstantCommand(this::doAutoSetup),
    //   TrajectoryCommands.getRamseteCommand(new Pose2d(0, 0, new Rotation2d(0)), new ArrayList<Translation2d>(), new Pose2d(3, 0, new Rotation2d(0)), drivetrain),
    //   new InstantCommand(() -> drivetrain.tankDriveVolts(0,0))
    // ));

    autoCommands = new SelectCommand(commands, this::getAutoId);

    autoChooser.setDefaultOption("example", AutoPaths.Example);

    SmartDashboard.putData(autoChooser);

    doTeleopSetup();
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

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   return null;
  // }

  public void doTeleopSetup() {
    drivetrain.setDefaultCommand(new DrivingCommand(drivetrain, () -> driverController.getRawAxis(DRIVER_LEFT_JOYSTICK_X_AXIS), 
    () -> driverController.getRawAxis(DRIVER_LEFT_JOYSTICK_Y_AXIS), () -> driverController.getRawAxis(DRIVER_RIGHT_JOYSTICK_Y_AXIS)));

    drivetrain.setDriveTrainVariables();

    setTeleop();
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
