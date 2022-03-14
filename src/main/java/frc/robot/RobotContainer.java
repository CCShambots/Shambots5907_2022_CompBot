package frc.robot;

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
import frc.robot.commands.intake.EjectBallCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.commands.turret.OdometryTurretTracking;
import frc.robot.commands.turret.ShootCommand;
import frc.robot.commands.turret.SpinUpFlywheelCommand;
import frc.robot.commands.turret.limelight.TeleopTrackingCommand;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Climber.ClimberState;
import frc.robot.subsystems.Climber.MotorSide;
import frc.robot.subsystems.Turret.Direction;
import frc.robot.util.auton.AutoRoutes;
import frc.robot.util.auton.AutoRoutes.AutoPaths;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import static frc.robot.Constants.Controller.*;
import static frc.robot.subsystems.Drivetrain.*;
import static frc.robot.Constants.Turret.*;

public class RobotContainer {
  ShuffleboardTab driveTab = Shuffleboard.getTab("Drive Team");
  ShuffleboardTab configurationTab = Shuffleboard.getTab("Configuration");

  Field2d field = new Field2d();
  
  private final Drivetrain drivetrain = new Drivetrain(driveTab);
  private final Intake intake = new Intake();
  private final Conveyor conveyor = new Conveyor();
  private final Turret turret = new Turret(driveTab);
  private final Climber climber = new Climber();

  TeleopTrackingCommand limeLightTeleopCommand = null;

  private final Joystick driverController = new Joystick(DRIVER_CONTROLLER_PORT);//makes new Driver Controller Object
  private final Joystick operatorController = new Joystick(OPERATOR_CONTROLLER_PORT);

  private AutoRoutes autoRoutes;
  Map<Object, Command> commands = new HashMap<>(); //The commands that will be chosen from in the sendable chooser
  SelectCommand autoCommand;
  SendableChooser<AutoPaths> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    //Put telemetry for choosing autonomous routes, displayign the field, and displaying the status of the robot
    driveTab.add(autoChooser);
    driveTab.addString("Robot Status", () -> getRobotStatus());
    driveTab.add(field);

    configureButtonBindings();
    
    //Load the different trajectories from their JSON files
    Map<String, Trajectory> paths = loadPaths(List.of( "CSGO1", "CSGO2", "CSGO31", "CSGO32", "BackUpLeftRoute", "BackUpMidRoute", "BackUpRightRoute"));

    //This object uses the trajectories to initialize each autonomous route command
    autoRoutes = new AutoRoutes(paths, drivetrain, intake, conveyor, turret);

    commands = autoRoutes.getAutoRoutes();

    autoCommand = new SelectCommand(commands, this::getAutoId);

    //Set up the sendable chooser for selecting different autonomous routes
    autoChooser.setDefaultOption("CSGO Left", AutoPaths.CSGO1);
    autoChooser.addOption("CSGO Mid", AutoPaths.CSGO2);

    //DO NOT UNCOMMENT!!!!!!!!
    // autoChooser.addOption("CSGO Right", AutoPaths.CSGO3);
    autoChooser.addOption("Back up Left", AutoPaths.BackUpLeft);
    autoChooser.addOption("Back up Mid", AutoPaths.BackUpMid);
    autoChooser.addOption("Back up Right", AutoPaths.BackUpRight);

    //TODO: Lights for if the turret is allowed to shoot or not
  }

  private void configureButtonBindings() {
    //Drivetrain controls

      //Speed changing
      new JoystickButton(driverController, Button.kRightBumper.value)
        .whenPressed(new InstantCommand(() -> drivetrain.setSpeed(TeleopSpeeds.Turbo)))
        .whenReleased(new InstantCommand(() -> drivetrain.setSpeed(TeleopSpeeds.Normal)));

      //Switch between tank and arcade drive
      new JoystickButton(driverController, Button.kLeftBumper.value)
        .whenPressed(new InstantCommand(drivetrain::toggleDriveMode));


    //Intake/Conveyor controls
    
      //Runs the intake command if the robot has fewer than two balls and the turret is not trying to shoot
      new JoystickButton(operatorController, 4)
        .whenPressed(new ConditionalCommand(new IntakeCommand(intake, conveyor),
        new InstantCommand(), () ->  {
          boolean allowed = (conveyor.getNumberOfBalls() < 2) && (limeLightTeleopCommand == null) && (!conveyor.isEjecting());
          SmartDashboard.putBoolean("Intake command allowed", allowed);
          System.out.println("conveyor command equals null " + (!conveyor.isEjecting()));
          System.out.println("allowed: " + allowed);
          return allowed;
        }));
      
      //Indicates that the intake command should end prematurely 
      //(balls will still be processed if they are still moving through the conveyor)
      new JoystickButton(operatorController, 1)
        .whenPressed(new InstantCommand(() -> intake.setShouldEnd(true)));

      //Ejects balls from the conveyor
      new JoystickButton(operatorController, 2)
        .whenPressed(new EjectBallCommand(conveyor, intake, 1.5), false);
      

    //Turret Controls
    
      //Begin or cancel tracking the central target with the target
      new JoystickButton(operatorController, 7)
        .whenPressed(new ConditionalCommand(new InstantCommand(() -> startLimelightTargeting(new TeleopTrackingCommand(turret, conveyor, intake))), new InstantCommand(), () -> conveyor.getNumberOfBalls() > 0 && turret.knowsLocation()))
        .whenReleased(new ConditionalCommand(new InstantCommand(() -> endLimelightTargeting()), new InstantCommand(), () -> limeLightTeleopCommand != null));
      
      //Only let the turret shoot  if the conveyor doesn't have a command
      new JoystickButton(operatorController, 9)
        .whenPressed(new InstantCommand(() -> turret.setShouldShoot(true)))
        .whenReleased(new InstantCommand(() -> turret.setShouldShoot(false)));
      
      //Switch the direction the turret will use to search for the target when it is not visible
      new JoystickButton(operatorController, 6).whenPressed(new InstantCommand(() -> turret.setSearchDirection(Direction.CounterClockwise)));
      new JoystickButton(operatorController, 10).whenPressed(new InstantCommand(() -> turret.setSearchDirection(Direction.Clockwise)));
        
      //Moves the turret to facing directly forward
      new JoystickButton(operatorController, 11)
        .whenPressed(new ConditionalCommand(new InstantCommand(() -> turret.setSpinnerTarget(0)), new InstantCommand(),
        () -> {
          return 
            limeLightTeleopCommand == null;
        }));
        
      //Spin up the flywheel and shoot into the low goal
      new JoystickButton(operatorController, 12)
        .whenPressed(new ConditionalCommand(new SequentialCommandGroup(
          new SpinUpFlywheelCommand(turret, Constants.Turret.FLYWHEEL_LOW_RPM),
          new ShootCommand(conveyor),
          new InstantCommand(() -> turret.setFlywheelTarget(0))
        ), new InstantCommand(), () -> {
          return limeLightTeleopCommand == null && conveyor.getNumberOfBalls() > 0;
        }));

      //Allow for very slow, manual movement of the turret in the event of a crash
      new JoystickButton(operatorController, 13)
        .whenPressed(new InstantCommand(() -> turret.setManualPower(MANUAL_SPEED)))
        .whenReleased(new InstantCommand(() -> turret.setManualPower(0)));

      new JoystickButton(operatorController, 14)
        .whenPressed(new InstantCommand(() -> turret.setManualPower(-MANUAL_SPEED)))
        .whenReleased(new InstantCommand(() -> turret.setManualPower(0)));


    //Climber controls
    new JoystickButton(operatorController, 3)
      .whenPressed(new MoveClimberCommand(climber, ClimberState.Lowered, drivetrain));
    
    new JoystickButton(operatorController, 5)
      .whenPressed(new MoveClimberCommand(climber, ClimberState.Mid, drivetrain));

    //Manual commands for moving the climber in for tim
    configurationTab.add("Raise Right Climber", climber.moveMotor(0.15, MotorSide.Right, false));
    configurationTab.add("Lower Right Climber", climber.moveMotor(-0.15, MotorSide.Right, true));
    configurationTab.add("Raise Left Climber", climber.moveMotor(0.15, MotorSide.Left, false));
    configurationTab.add("Lower Left Climber", climber.moveMotor(-0.15, MotorSide.Left, true));

    driveTab.add("DISABLE TURRET TRACKING", new InstantCommand(() -> turret.setKnowsLocation(false)));
    driveTab.addBoolean("TURRET CAN TRACK", () -> turret.knowsLocation());
    
    //Soft e-stop that cancels all subsystem commands and should stop motors from moving.
    new JoystickButton(operatorController, 8)
      .whenPressed(new InstantCommand(
        () -> {
          if(conveyor.getCurrentCommand() != null) conveyor.getCurrentCommand().cancel();

          intake.stop();
          conveyor.stopAll();
          turret.setFlywheelTarget(0);
          turret.setLimelightOff();
          turret.setSpinnerTarget(turret.getSpinnerAngle());

          conveyor.setEjecting(false);
          climber.brake();
        }
      , intake, turret, climber
      ));
  }

  public void telemetry() {
    //TODO: Remove all this telemtry once we don't need it (other telemetry found in periodic() of subsystems)

    //Drivetrain telemetry
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

    if(Constants.robotStatus != RobotStatus.DISABLED) {
      field.setRobotPose(drivetrain.getOdometryPose());
      
      SmartDashboard.putData(field);

    } else {
      //Display the robot's starting pose to the drivers before the beginning of autonomous
      Trajectory firstTrajectory = autoRoutes.getFirstTrajectory(autoChooser.getSelected());
      field.setRobotPose(firstTrajectory.getInitialPose());
    }

    SmartDashboard.putNumber("robot x", drivetrain.getOdometryPose().getX());
    SmartDashboard.putNumber("robot y", drivetrain.getOdometryPose().getY());
    SmartDashboard.putNumber("robot z", drivetrain.getOdometryPose().getRotation().getDegrees());

    SmartDashboard.putBoolean("Conveyor command equals null", conveyor.getCurrentCommand() == null);
  }

  public void toggleLimelightTargeting() {
    if(limeLightTeleopCommand != null) {
      endLimelightTargeting();
    } else if (conveyor.getNumberOfBalls() > 0){ //Only allow the command to begin if there are balls in the robot
      startLimelightTargeting(new TeleopTrackingCommand(turret, conveyor, intake));
    }
  }

  public void startLimelightTargeting(TeleopTrackingCommand command) {
    limeLightTeleopCommand = command;
    
    limeLightTeleopCommand.schedule(false);
  }
  
  public void endLimelightTargeting() {
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
  public boolean isShootingAllowed() {
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
  }

  public void resetDrivetrainPID() {
    drivetrain.resetPID();
  }

  
  public void doTurretSetup() {
    turret.setDefaultCommand(new OdometryTurretTracking(drivetrain, conveyor, turret));
    turret.resetSpinnerPID();
    turret.setSpinnerTarget(turret.getSpinnerAngle());
    turret.setFlywheelTarget(0);
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

  public void resetClimber() {climber.resetClimber();}

  private String getRobotStatus() {return Constants.robotStatus.name();}

  public SelectCommand getAutoCommand() {
    return autoCommand;
  }

  public Command getLimelightCommand() {return limeLightTeleopCommand;}

  public AutoPaths getAutoId() {
    System.out.println("Auto id" + autoChooser.getSelected());
    return autoChooser.getSelected();
  }

  public void setAutonomous() {
    Constants.robotStatus = RobotStatus.AUTO;
    drivetrain.setNeutralMotorBehavior(NeutralMode.Brake);
    turret.setSpinnerNeutralMode(NeutralMode.Brake);
  }

  public void setTeleop() {
    Constants.robotStatus = RobotStatus.TELEOP;
    drivetrain.setNeutralMotorBehavior(NeutralMode.Brake);
    turret.setSpinnerNeutralMode(NeutralMode.Brake);
  }

  public void setDisabled() {
    Constants.robotStatus = RobotStatus.DISABLED;
  }

  public void setTest() {
    Constants.robotStatus = RobotStatus.TEST;
    drivetrain.setNeutralMotorBehavior(NeutralMode.Coast);
    turret.setSpinnerNeutralMode(NeutralMode.Coast);
  }

  public static enum RobotStatus {
    AUTO, TELEOP, DISABLED, TEST
  } 
}
