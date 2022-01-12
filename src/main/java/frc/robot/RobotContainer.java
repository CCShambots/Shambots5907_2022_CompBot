// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  private final Drivetrain drivetrain = new Drivetrain();
  private final Joystick driverController = new Joystick(Constants.DRIVER_CONTROLLER_PORT);//makes new Driver Controller Object

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public RobotContainer() {
    configureButtonBindings();

    //Set always running command for teleop on the drivetrain (basic drive control)
    drivetrain.setDefaultCommand(new RunCommand(() -> {
        drivetrain.tankDrive(driverController.getRawAxis(Constants.DRIVER_LEFT_JOYSTICK_Y_AXIS), 
        driverController.getRawAxis(Constants.DRIVER_RIGHT_JOYSTICK_Y_AXIS));
    }, drivetrain));
  }

  private void configureButtonBindings() {
    //Add button for setting turbo speed when a certain button is held
    new JoystickButton(driverController, Constants.DRIVER_BUTTON_6)
      .whenPressed(new InstantCommand(drivetrain::setTurboSpeed))
      .whenReleased(new InstantCommand(drivetrain::setNormalSpeed));
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

  public void doTeleopInit() {
    drivetrain.teleopInit();
  }

  public void doOdoUpdate() {
    drivetrain.updateOdometry();
  }
}
