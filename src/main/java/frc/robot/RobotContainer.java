package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.LimeLightTurnCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
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

import java.util.HashMap;
import java.util.Map;

import static frc.robot.Constants.Controller.*;
import static frc.robot.Constants.Limelight.*;

public class RobotContainer {
  private final Drivetrain drivetrain = new Drivetrain();
  private final Limelight limelight = new Limelight();

  private final Joystick driverController = new Joystick(DRIVER_CONTROLLER_PORT);//makes new Driver Controller Object
  private final Joystick operatorController = new Joystick(OPERATOR_CONTROLLER_PORT);

  //TODO: Make this based on the chosen auto route (idk how but it needs to be done!)
  private Pose2d startPose = new Pose2d();

  private SelectCommand autoCommands;
  private Map<Object, Command> commands = new HashMap<>();

  //Sendable chooser for picking an auton route
  SendableChooser<AutoPaths> autoChooser = new SendableChooser<>();


  public RobotContainer() {
    configureButtonBindings();

    commands.put(Example, new SequentialCommandGroup(
      new InstantCommand(this::doAutoSetup)
    ));

    autoCommands = new SelectCommand(commands, this::getAutoId);

    autoChooser.setDefaultOption("example", Example);

    SmartDashboard.putData(autoChooser);
  }

  private void configureButtonBindings() {
    //Add button for setting turbo speed when a certain button is held
    new JoystickButton(operatorController, OPERATOR_BUTTON_6)
      .whenPressed(new ConditionalCommand(new InstantCommand(() -> drivetrain.setSpeed(Turbo)), new InstantCommand(), drivetrain::isToggleDriveModeAllowed))
      .whenReleased(new ConditionalCommand(new InstantCommand(() -> drivetrain.setSpeed(Normal)), new InstantCommand(), drivetrain::isToggleDriveModeAllowed));

    new JoystickButton(operatorController, OPERATOR_BUTTON_3)
      .whenPressed(new InstantCommand(() -> drivetrain.setDriveMode(DriveModes.Limelight)))
      .whenReleased(new InstantCommand(() -> drivetrain.setDriveMode(drivetrain.getPrevDriveMode())));

    new JoystickButton(operatorController, OPERATOR_BUTTON_5)
      .whenPressed(new InstantCommand(drivetrain::toggleDriveMode));
      
    new JoystickButton(operatorController, OPERATOR_BUTTON_4)
      .whenPressed(new InstantCommand(drivetrain::toggleReversed));
  }

  public void doTeleopSetup() {

    //TODO: Decide if this is necessary
    drivetrain.setDampening(1);

    //The different drive modes for the robot, which control whether the robot is in tank drive, arcade drive, or limelight orienting mode
    drivetrain.setDefaultCommand(
      new SelectCommand(
        new HashMap<Object, Command>() {{
          put(DriveModes.Tank, new RunCommand(() -> {
            drivetrain.tankDrive(
              driverController.getRawAxis(DRIVER_LEFT_JOYSTICK_Y_AXIS), 
              driverController.getRawAxis(DRIVER_RIGHT_JOYSTICK_Y_AXIS));
            }));
          put(DriveModes.Arcade, new RunCommand(() -> {
            drivetrain.arcadeDrive(
              driverController.getRawAxis(DRIVER_LEFT_JOYSTICK_Y_AXIS), 
              driverController.getRawAxis(DRIVER_LEFT_JOYSTICK_X_AXIS));
          }));
          put(DriveModes.Limelight, new LimeLightTurnCommand(limelight, drivetrain, Z_LIMELIGHT_PID));
        }},
        drivetrain::getDriveMode)
    );

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
}
