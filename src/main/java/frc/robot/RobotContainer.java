// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DrivingCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.HashMap;
import java.util.Map;

import static frc.robot.Constants.Controller.*;
import static frc.robot.subsystems.Drivetrain.*;

public class RobotContainer {
  Field2d field = new Field2d();
  private final Drivetrain drivetrain = new Drivetrain();
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
    //Add button for setting turbo speed when a certain button is held
    new JoystickButton(driverController, DRIVER_A)
      .whenPressed(new ConditionalCommand(new InstantCommand(() -> drivetrain.setSpeed(TeleopSpeeds.Turbo)), new InstantCommand(), drivetrain::isToggleDriveModeAllowed))
      .whenReleased(new ConditionalCommand(new InstantCommand(() -> drivetrain.setSpeed(TeleopSpeeds.Normal)), new InstantCommand(), drivetrain::isToggleDriveModeAllowed));

    new JoystickButton(driverController, DRIVER_B)
      .whenPressed(new InstantCommand(drivetrain::toggleDriveMode));
      
    new JoystickButton(driverController, DRIVER_X)
      .whenPressed(new InstantCommand(drivetrain::toggleReversed));


    //TODO: Should the intake continue running until stopped with a different button, or just run while held down
    //TODO: Prioritize running the inatke forwards or backwards
    new JoystickButton(driverController, OPERATOR_4_2)
      .whenPressed(new InstantCommand(intake::intakeBalls))
      .whenReleased(new InstantCommand(intake::stopIntake));
      
    new JoystickButton(driverController, OPERATOR_4_3)
      .whenPressed(new InstantCommand(intake::exhaustBalls))
      .whenReleased(new InstantCommand(intake::stopIntake));

    new JoystickButton(driverController, OPERATOR_2_1)
      .whenPressed(new InstantCommand(intake::raiseIntake));
      
    new JoystickButton(driverController, OPERATOR_3_1)
      .whenPressed(new InstantCommand(intake::lowerIntake));
    
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
    //TODO: Decide if this is necessary
    // drivetrain.setDampening(1);x

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
