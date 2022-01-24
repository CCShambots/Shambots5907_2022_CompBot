package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DrivingCommand;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.HashMap;
import java.util.Map;

import static frc.robot.Constants.Controller.*;
import static frc.robot.subsystems.Drivetrain.*;

public class RobotContainer {
  private final Drivetrain drivetrain = new Drivetrain();

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
    new JoystickButton(operatorController, OPERATOR_A)
      .whenPressed(new ConditionalCommand(new InstantCommand(() -> drivetrain.setSpeed(TeleopSpeeds.Turbo)), new InstantCommand(), drivetrain::isToggleDriveModeAllowed))
      .whenReleased(new ConditionalCommand(new InstantCommand(() -> drivetrain.setSpeed(TeleopSpeeds.Normal)), new InstantCommand(), drivetrain::isToggleDriveModeAllowed));

    new JoystickButton(operatorController, OPERATOR_B)
      .whenPressed(new InstantCommand(drivetrain::toggleDriveMode));
      
    new JoystickButton(operatorController, OPERATOR_X)
      .whenPressed(new InstantCommand(drivetrain::toggleReversed));
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
  }

  public void doTeleopSetup() {

    //TODO: Decide if this is necessary
    // drivetrain.setDampening(1);x

    drivetrain.setDefaultCommand(new DrivingCommand(drivetrain, () -> driverController.getRawAxis(DRIVER_LEFT_JOYSTICK_X_AXIS), 
    () -> driverController.getRawAxis(DRIVER_LEFT_JOYSTICK_Y_AXIS), () -> driverController.getRawAxis(DRIVER_RIGHT_JOYSTICK_Y_AXIS)));

    // drivetrain.setDefaultCommand(
    //   new RunCommand(
    //     () -> drivetrain.tankDrivePID(
    //       driverController.getRawAxis(DRIVER_LEFT_JOYSTICK_Y_AXIS), 
    //       driverController.getRawAxis(DRIVER_RIGHT_JOYSTICK_Y_AXIS)), 
    //    drivetrain)
    // );

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
