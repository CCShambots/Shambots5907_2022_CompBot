// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.AutonomousCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.AutoPaths;
import frc.robot.util.DriveModes;
import frc.robot.util.RobotStatus;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static frc.robot.util.TeleopSpeeds.*;
import static frc.robot.util.AutoPaths.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class RobotContainer {
  private final Drivetrain drivetrain = new Drivetrain();
  private final Joystick driverController = new Joystick(Constants.DRIVER_CONTROLLER_PORT);//makes new Driver Controller Object

  //TODO: Make this based on the chosen auto route (idk how but it needs to be done!)
  private Pose2d startPose = new Pose2d();

  // private List<AutonomousCommand> autoCommands = new ArrayList<>();
  private AutoPaths autoId = Example;
  private SelectCommand autoCommands;
  Map<Object, Command> commands = new HashMap<>();

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public RobotContainer() {
    configureButtonBindings();

    commands.put(Example, new SequentialCommandGroup(
      new InstantCommand(this::doAutoSetup)
    ));

    autoCommands = new SelectCommand(commands, this::getAutoId);
  }

  private void configureButtonBindings() {
    //Add button for setting turbo speed when a certain button is held
    new JoystickButton(driverController, Constants.DRIVER_BUTTON_6)
      .whenPressed(new InstantCommand(() -> drivetrain.setSpeed(Turbo)))
      .whenReleased(new InstantCommand(() -> drivetrain.setSpeed(Normal)));

    new JoystickButton(driverController, Constants.DRIVER_BUTTON_5)
      .whenPressed(new InstantCommand(drivetrain::toggleDriveMode));
      
    new JoystickButton(driverController, Constants.DRIVER_BUTTON_4)
      .whenPressed(new InstantCommand(drivetrain::toggleReversed));
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

    //Set always running command for teleop on the drivetrain (basic drive control)
    drivetrain.setDefaultCommand(
      new SelectCommand(
        new HashMap<Object, Command>() {{
          put(DriveModes.Tank, new RunCommand(() -> {
            drivetrain.tankDrive(
              driverController.getRawAxis(Constants.DRIVER_LEFT_JOYSTICK_Y_AXIS), 
              driverController.getRawAxis(Constants.DRIVER_RIGHT_JOYSTICK_Y_AXIS));
            }));
          put(DriveModes.Arcade, new RunCommand(() -> {
            drivetrain.arcadeDrive(
              driverController.getRawAxis(Constants.DRIVER_LEFT_JOYSTICK_Y_AXIS), 
              driverController.getRawAxis(Constants.DRIVER_LEFT_JOYSTICK_X_AXIS));
          }));
          
        }},
        drivetrain::getDriveMode)
    );

    setTeleop();
  }

  public SelectCommand getAutoCommand() {
    return autoCommands;
  }

  public AutoPaths getAutoId() {
    return autoId;
  }

  public void setAutoId(AutoPaths id) {
    autoId = id;
  }

  public void doAutoSetup() {
    setAutonomous();
    drivetrain.resetOdometry(startPose);
  }

  public void setAutonomous() {
    Constants.robotStatus = RobotStatus.AUTO;
  }

  public void setTeleop() {
    Constants.robotStatus = RobotStatus.TELEOP;
  }

  public void setDisabled() {
    Constants.robotStatus = RobotStatus.DISABLED;
  }
}
