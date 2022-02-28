// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.commands.intake.ReleaseBallCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.commands.limelight.BasicTrackingCommand;
import frc.robot.commands.limelight.TeleopTrackingCommand;
import frc.robot.commands.turret.ShootCommand;
import frc.robot.commands.turret.ShootCommand.Amount;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Turret.Direction;
import frc.robot.util.auton.AutoRoutes;
import frc.robot.util.auton.AutoRoutes.AutoPaths;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import static frc.robot.Constants.Controller.*;
import static frc.robot.subsystems.Drivetrain.*;

public class RobotContainer {
  ShuffleboardTab driveTab = Shuffleboard.getTab("Drive Team");

  Field2d field = new Field2d();

  private final Drivetrain drivetrain = new Drivetrain(driveTab);
  private final Intake intake = new Intake();
  private final Conveyor conveyor = new Conveyor();
  private final Turret turret = new Turret(driveTab);
  // private final Climber climber = new Climber();

  TeleopTrackingCommand limeLightTeleopCommand = null;

  private final Joystick driverController = new Joystick(DRIVER_CONTROLLER_PORT);//makes new Driver Controller Object
  private final Joystick operatorController = new Joystick(OPERATOR_CONTROLLER_PORT);

  private SelectCommand autoCommands;
  Map<Object, Command> commands = new HashMap<>(); //The commands that will be chosen from in the sendable chooser
  SendableChooser<AutoPaths> autoChooser = new SendableChooser<>();


  public RobotContainer() {
    //Put telemetry for choosing autonomous routes, displayign the field, and displaying the status of the robot
    driveTab.add(autoChooser);
    driveTab.add("field", field);
    driveTab.addString("Robot Status", () -> getRobotStatus());

    configureButtonBindings();
    
    //Load the different trajectories from their JSON files
    Map<String, Trajectory> paths = loadPaths(List.of( "CSGO1", "CSGO2", "CSGO31", "CSGO32"));

    //This object uses the trajectories to initialize each autonomous route command
    // AutoRoutes autoRoutes = new AutoRoutes(paths, drivetrain, intake, conveyor, turret);

    // commands = autoRoutes.getAutoRoutes();

    autoCommands = new SelectCommand(commands, this::getAutoId);

    //Set up the sendable chooser for selecting different autonomous routes
    autoChooser.setDefaultOption("CSGO-1", AutoPaths.CSGO1);
    autoChooser.addOption("CSGO-2", AutoPaths.CSGO2);
    autoChooser.addOption("CSGO-3", AutoPaths.CSGO3);

    //TODO: Telemetry for if the turret is allowed to shoot or not

    doDrivetrainSetup();
  }

  private void configureButtonBindings() {
    //TODO: Disable Intake commands when the turret is shooting

    //Drivetrain controls

    //Speed changing
    new JoystickButton(driverController, Button.kRightBumper.value)
      .whenPressed(new InstantCommand(() -> drivetrain.setSpeed(TeleopSpeeds.Turbo)))
      .whenReleased(new InstantCommand(() -> drivetrain.setSpeed(TeleopSpeeds.Normal)));

    //Switch between tank and arcade drive
    new JoystickButton(driverController, Button.kLeftBumper.value)
      .whenPressed(new ConditionalCommand(new InstantCommand(drivetrain::toggleDriveMode), new InstantCommand(), drivetrain::isToggleDriveModeAllowed));


    //Intake/Conveyor controls
    
    //Runs the intake command if the robot has fewer than two balls and the turret is not trying to shoot
    new JoystickButton(operatorController, 4)
      .whenPressed(new ConditionalCommand(new IntakeCommand(intake, conveyor),
      new InstantCommand(), () -> (conveyor.getNumberOfBalls() < 2) && (limeLightTeleopCommand == null)));
    
    //Indicates that the intake command should end prematurely 
    //(balls will still be processed if they are still moving through the conveyor)
    new JoystickButton(operatorController, 1)
      .whenPressed(new InstantCommand(() -> intake.setShouldEnd(true)));

    //Ejects balls from the conveyor
    //TOOD: Make this better
    new JoystickButton(operatorController, 2)
      .whenPressed(new SequentialCommandGroup(new InstantCommand(() -> {
        conveyor.exhaustAll();
        conveyor.setTrackerDisabled(true);
      }, conveyor),
      new WaitCommand(1),
      new InstantCommand(() -> {
        conveyor.stopAll();
        conveyor.clearTracker();
        conveyor.setTrackerDisabled(false);
      }, conveyor)));
    

    //Turret Controls
    
    //Begin or cancel tracking the central target with the target
    new JoystickButton(driverController, Button.kB.value)
      .whenPressed(new InstantCommand(() -> toggleLimelightTargeting()));
    
    //Only let the turret shoot  if the conveyor doesn't have a command
    new JoystickButton(driverController, Button.kA.value)
      .whenPressed(new ConditionalCommand(new ShootCommand(conveyor, Amount.Two, this), new InstantCommand(), this::isShootingAllowed));
    
    //Switch the direction the turret will use to search for the target when it is not visible
    new JoystickButton(operatorController, 6).whenPressed(new InstantCommand(() -> turret.setSearchDirection(Direction.CounterClockwise)));
    new JoystickButton(operatorController, 7).whenPressed(new InstantCommand(() -> turret.setSearchDirection(Direction.Clockwise)));
    
    //TODO: Remove this
    new JoystickButton(driverController, Button.kY.value).whenPressed(new InstantCommand(() -> {
        turret.setFlywheelTarget(4250);
        conveyor.intakeAll();
      })
      ).whenReleased(new InstantCommand(() ->  {
        turret.setFlywheelTarget(0);
        conveyor.stopAll();
      }));
    // new JoystickButton(driverController, Button.kX.value).whenPressed(new InstantCommand(() -> turret.setSpinnerTarget(5))).whenReleased(new InstantCommand(() -> turret.setSpinnerTarget(0)));
    
    //TODO: Add these back in later
    // //Climber controls
    // new JoystickButton(operatorController, 3)
    //   .whenPressed(new MoveClimberCommand(climber, ClimberState.Lowered));
    
    // new JoystickButton(operatorController, 5)
    //   .whenPressed(new MoveClimberCommand(climber, ClimberState.Mid));

    //TODO: Test the soft estop (lest you die at kettering >:())
    //TODO: Add climber back to this
    //Soft e-stop that cancels all subsystem commands and should stop motors from moving.
    new JoystickButton(operatorController, 8)
      .whenPressed(new InstantCommand(
        () -> {
          intake.stop();
          conveyor.stopAll();
          turret.setFlywheelTarget(0);
          turret.setLimelightOff();
          turret.setSpinnerTarget(turret.getSpinnerAngle());
          // climber.brake();
        }
      , intake, conveyor, turret //, climber
      ));
  }

  public void telemetry() {
    //TODO: Remove all this telemtry once we don't need it (other telemetry found in periodic() of subsystems)

    //Drivetrain telemetry
    // SmartDashboard.putNumber("Gyro value", drivetrain.getGyroHeading());
    // SmartDashboard.putNumber("Left meters traveled", drivetrain.getLeftMeters());
    // SmartDashboard.putNumber("Right meters traveled", drivetrain.getRightMeters());
    // SmartDashboard.putNumber("Left voltage", drivetrain.getLeftVoltage());
    // SmartDashboard.putNumber("Right voltage", drivetrain.getRightVoltage());
    // SmartDashboard.putNumber("Left velocity", drivetrain.getLeftVelocity());
    // SmartDashboard.putNumber("Right velocity", drivetrain.getRightVelocity());
    // SmartDashboard.putNumber("Left feed forward", drivetrain.getLeftModule().getFeedForwardOutput());
    // SmartDashboard.putNumber("Right feed forward", drivetrain.getRightModule().getFeedForwardOutput());
    // SmartDashboard.putNumber("Left PID", drivetrain.getLeftModule().getPIDOutput());
    // SmartDashboard.putNumber("Right PID", drivetrain.getRightModule().getPIDOutput());
    // SmartDashboard.putData("RightPID", drivetrain.getRightModule().getPIDController());
    // SmartDashboard.putData("leftPID", drivetrain.getLeftModule().getPIDController());
    // SmartDashboard.putNumber("left setpoint", drivetrain.getLeftModule().getSetpoint());
    // SmartDashboard.putNumber("right setpoint", drivetrain.getRightModule().getSetpoint());
    // field.setRobotPose(drivetrain.getOdometryPose());
    // SmartDashboard.putNumber("robot x", drivetrain.getOdometryPose().getX());
    // SmartDashboard.putNumber("robot y", drivetrain.getOdometryPose().getY());
    // SmartDashboard.putNumber("robot z", drivetrain.getOdometryPose().getRotation().getDegrees());
  }

  public void toggleLimelightTargeting() {
    if(limeLightTeleopCommand != null) {
      endLimelightTargeting();
    } else if (conveyor.getNumberOfBalls() > 0){ //Only allow the command to begin if there are balls in the robot
      startLimelightTargeting(new TeleopTrackingCommand(turret, conveyor));
    }
  }

  private void startLimelightTargeting(TeleopTrackingCommand command) {
    limeLightTeleopCommand = command;
    
    limeLightTeleopCommand.schedule(false);
  }
  
  private void endLimelightTargeting() {
    limeLightTeleopCommand.cancel();
    
    limeLightTeleopCommand = null;  
  }
  
  /**
   * Function that evaluates if the robot is in a state where it is able to shoot
   * Requirements:
   *  - the limelight is tracking
   *  - the intake does not have another command running
   *  - the limelight is locked in (i.e. right on target)
   *  - the flywheel is at the right speed
   * @return
   */
  private boolean isShootingAllowed() {
    if(limeLightTeleopCommand == null) return false;
    if(intake.getCurrentCommand() != null) return false;

    return limeLightTeleopCommand.isReady();
  }

  public void doDrivetrainSetup() {
    //Set the default command for easy driving
    drivetrain.setDefaultCommand(new DrivingCommand(drivetrain, () -> driverController.getRawAxis(DRIVER_LEFT_JOYSTICK_X_AXIS), 
    () -> driverController.getRawAxis(DRIVER_LEFT_JOYSTICK_Y_AXIS), () -> driverController.getRawAxis(DRIVER_RIGHT_JOYSTICK_Y_AXIS)));
    
    //This updates variables from the dashbaord sliders
    drivetrain.setDriveTrainVariables();
    
    setTeleop();
    
    //TODO: Disable resetting odometry if (and when) we are no longer using it in teleop
    drivetrain.resetOdometry(new Pose2d(13.5, 27.0, new Rotation2d(0.0)));
  }
  
  /**
   * Loads the different trajectories from built Pathweaver JSON files
   * @param names A list of the different trajectories to load
   * @return A map of the loaded trajectories
   */
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
    }

    return trajectories;
  }
  
  public void disableLimelight() { turret.setLimelightOff();}
  public void enableLimelight() { turret.setLimelightOn();}

  public void raiseIntake() {intake.raiseIntake();}

  private String getRobotStatus() {return Constants.robotStatus.name();}

  public SelectCommand getAutoCommand() {return autoCommands;}

  public AutoPaths getAutoId() {return autoChooser.getSelected();}

  public void setAutonomous() {
    Constants.robotStatus = RobotStatus.AUTO;
    drivetrain.setNeutralMotorBehavior(NeutralMode.Brake);
    turret.setBrake(NeutralMode.Brake);
  }

  public void setTeleop() {
    Constants.robotStatus = RobotStatus.TELEOP;
    drivetrain.setNeutralMotorBehavior(NeutralMode.Brake);
    turret.setBrake(NeutralMode.Brake);
  }

  public void setDisabled() {
    Constants.robotStatus = RobotStatus.DISABLED;
  }

  public void setTest() {
    Constants.robotStatus = RobotStatus.TEST;
    drivetrain.setNeutralMotorBehavior(NeutralMode.Coast);
    turret.setBrake(NeutralMode.Coast);
  }

  public static enum RobotStatus {
    AUTO, TELEOP, DISABLED, TEST
  } 
}
