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
import frc.robot.commands.SoftStop;
import frc.robot.commands.climber.MoveClimberCommand;
import frc.robot.commands.drivetrain.DrivingCommand;
import frc.robot.commands.intake.EjectBallCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.commands.turret.LowGoalShootCommand;
import frc.robot.commands.turret.OdometryTurretTracking;
import frc.robot.commands.turret.limelight.TeleopTrackingCommand;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Climber.ClimberState;
import frc.robot.subsystems.Climber.MotorSide;
import frc.robot.subsystems.Turret.ControlLoop;
import frc.robot.subsystems.Turret.Direction;
import frc.robot.util.auton.AutoRoutes;
import frc.robot.util.auton.AutoRoutes.AutoPaths;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.priorityFramework.NotACommandException;
import frc.robot.util.priorityFramework.PriorityCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
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
import static frc.robot.util.priorityFramework.PriorityRegistry.*;

public class RobotContainer {
  ShuffleboardTab driveTab = Shuffleboard.getTab("Drive Team");
  ShuffleboardTab configurationTab = Shuffleboard.getTab("Configuration");

  Field2d field = new Field2d();
  
  private final Drivetrain drivetrain = new Drivetrain(driveTab);
  private final Intake intake = new Intake();
  private final Conveyor conveyor = new Conveyor();
  private final Turret turret = new Turret(driveTab);
  // private final Climber climber = new Climber();
  private final Lights lights = new Lights();

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

    
    //Load the different trajectories from their JSON files
    Map<String, Trajectory> paths = loadPaths(List.of( "CSGO1", "CSGO2", "CSGO31", "CSGO32", 
    "BackUpLeftRoute", "BackUpMidRoute", "BackUpRightRoute", "Meter", "FourBall1", "FourBall2", "FourBall3"));

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
    autoChooser.addOption("Meter", AutoPaths.Meter);


    //TODO: Lights for if the turret is allowed to shoot or not

    //Register priority comnmads
    try {
      registerCommand(LowGoalShootCommand.class, 1);
      registerCommand(IntakeCommand.class, 2);
      registerCommand(TeleopTrackingCommand.class, 2);
      registerCommand(EjectBallCommand.class, 3);
      registerCommand(SoftStop.class, 5);
    } catch (NotACommandException e) {
      e.printStackTrace();
    }

    configureButtonBindings();
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

      //TODO: What button should this be on?
      //Reverse the bot controls
      new JoystickButton(driverController, Button.kA.value)
        .whenPressed(new InstantCommand(drivetrain::toggleReversed));


    //Intake/Conveyor controls
    
      //Runs the intake command if the robot has fewer than two balls and the turret is not trying to shoot
      new JoystickButton(operatorController, 4)
        .whenPressed(new PriorityCommand(new IntakeCommand(intake, conveyor), () -> conveyor.getNumberOfBalls() < 2));
      
      //Indicates that the intake command should end prematurely 
      //(balls will still be processed if they are still moving through the conveyor)
      new JoystickButton(operatorController, 1)
        .whenPressed(new InstantCommand(() -> intake.setShouldEnd(true)));

      //Ejects balls from the conveyor
      new JoystickButton(operatorController, 2)
        .whenPressed(new PriorityCommand(new EjectBallCommand(conveyor, intake, 1.5)));
      

    //Turret Controls
    
      //Begin or cancel tracking the central target with the target
      new JoystickButton(operatorController, 7)
        .whenPressed(new PriorityCommand(new TeleopTrackingCommand(drivetrain, turret, conveyor), () -> conveyor.getNumberOfBalls() > 0))
        .whenReleased(new InstantCommand(() -> turret.setShouldEndTargeting(true)));
      
      //Only let the turret shoot  if the conveyor doesn't have a command
      new JoystickButton(operatorController, 9)
        .whenPressed(new InstantCommand(() -> turret.setShouldShoot(true)))
        .whenReleased(new InstantCommand(() -> turret.setShouldShoot(false)));
      
      //Switch the direction the turret will use to search for the target when it is not visible
      new JoystickButton(operatorController, 6).whenPressed(new InstantCommand(() -> turret.setSearchDirection(Direction.CounterClockwise)));
      new JoystickButton(operatorController, 10).whenPressed(new InstantCommand(() -> turret.setSearchDirection(Direction.Clockwise)));
      
      //Moves the turret to facing directly forward
      new JoystickButton(operatorController, 11)
        .whenPressed(new PriorityCommand(new InstantCommand(() -> turret.setSpinnerTarget(0)), 1));

      // new JoystickButton(driverController, Button.kA.value)
      //   .whenPressed(new InstantCommand(() -> turret.setSpinnerTarget(-90)));
      
      // new JoystickButton(driverController, Button.kB.value)
      //   .whenPressed(new InstantCommand(() -> turret.setSpinnerTarget(90)));

      new JoystickButton(driverController, Button.kA.value)
        .whenPressed(new InstantCommand(() -> {
          turret.setFlywheelControlLoop(ControlLoop.HighSpeed);
          turret.setFlywheelTarget(FLYWHEEL_HIGH_RPM);
        }))
        .whenReleased(new InstantCommand(() -> {
          turret.setFlywheelTarget(0);
        }));

      new JoystickButton(driverController, Button.kB.value)
      .whenPressed(new InstantCommand(() -> {
        turret.setFlywheelControlLoop(ControlLoop.LowSpeed);
        turret.setFlywheelTarget(FLYWHEEL_LOW_RPM);
      }))
      .whenReleased(new InstantCommand(() -> {
        turret.setFlywheelTarget(0);
      }));
        
      //Spin up the flywheel and shoot into the low goal
      new JoystickButton(operatorController, 12)
        .whenPressed(new PriorityCommand(new LowGoalShootCommand(conveyor, turret), () -> conveyor.getNumberOfBalls() > 0, turret));

      //Allow for very slow, manual movement of the turret in the event of a crash
      new JoystickButton(operatorController, 13)
        .whenPressed(new InstantCommand(() -> turret.setManualPower(MANUAL_SPEED)))
        .whenReleased(new InstantCommand(() -> turret.setManualPower(0)));

      new JoystickButton(operatorController, 14)
        .whenPressed(new InstantCommand(() -> turret.setManualPower(-MANUAL_SPEED)))
        .whenReleased(new InstantCommand(() -> turret.setManualPower(0)));


    //Climber controls
    // new JoystickButton(operatorController, 3)
      // .whenPressed(new MoveClimberCommand(climber, ClimberState.Lowered, drivetrain));
    
    // new JoystickButton(operatorController, 5)
      // .whenPressed(new MoveClimberCommand(climber, ClimberState.Mid, drivetrain));

    //Manual commands for moving the climber in for tim
    // configurationTab.add("Raise Right Climber", climber.moveMotor(0.15, MotorSide.Right, false));
    // configurationTab.add("Lower Right Climber", climber.moveMotor(-0.15, MotorSide.Right, true));
    // configurationTab.add("Raise Left Climber", climber.moveMotor(0.15, MotorSide.Left, false));
    // configurationTab.add("Lower Left Climber", climber.moveMotor(-0.15, MotorSide.Left, true));
    
    driveTab.add("Toggle Odomdetry tracking", new InstantCommand(() -> drivetrain.toggleUseOdometry()));
    driveTab.addBoolean("Odometry targeting active?", () -> drivetrain.shouldUseOdometry());

    //Soft stop that cancels all subsystem commands and should stop motors from moving.
    new JoystickButton(operatorController, 8)
      .whenPressed(new PriorityCommand(new SoftStop(intake, conveyor, turret, null)));
  }

  public void telemetry() {
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
  
    SmartDashboard.putData("subsystems/Drivetrain", drivetrain);
    SmartDashboard.putData("subsystems/Intake", intake);
    SmartDashboard.putData("subsystems/Conveyor", conveyor);
    SmartDashboard.putData("subsystems/Turret", turret);
    // SmartDashboard.putData("subsystems/Climber", climber);
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
    
    //TODO: REmove this at some point
    turret.setKnowsLocation(true);
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

  // public void resetClimber() {climber.resetClimber();}

  public void resetSubsystems() {
    drivetrain.resetPriority();
    intake.resetPriority();
    conveyor.resetPriority();
    turret.resetPriority();
    // climber.resetPriority();
  }

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
