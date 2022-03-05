package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    //Crates subsystems, loads trajectories, does button bindings, etc.
    m_robotContainer = new RobotContainer();

    m_robotContainer.disableLimelight();

    m_robotContainer.setDisabled();

    LiveWindow.disableAllTelemetry();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    //Update dashboard telemetry
    m_robotContainer.telemetry();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.setDisabled();
    m_robotContainer.disableLimelight();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.setAutonomous();

    m_robotContainer.getAutoCommand().schedule();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    //Update the robot's status so any subsystem that needs to know it's teleop can know
    m_robotContainer.setTeleop();
    //Setup the joysticks and pull the config values from the drivetrain from Shuffleboard
    m_robotContainer.doDrivetrainSetup();
    m_robotContainer.raiseIntake();
    m_robotContainer.doTurretSetup();
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.setTest();
    m_robotContainer.enableLimelight();
  }

  @Override
  public void testPeriodic() {}
}
