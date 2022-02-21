package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    //Crates subsystems, loads trajectories, does button bindings, etc.
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    //Update dashboard telemetry
    //TODO: Move this telemetry to the subsystems when I feel like it later
    m_robotContainer.telemetry();
  }

  @Override
  public void disabledInit() {
    new InstantCommand(() -> m_robotContainer.setDisabled()).schedule();
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
    m_robotContainer.setTeleop();
    m_robotContainer.doDrivetrainSetup();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.setTest();
  }

  @Override
  public void testPeriodic() {}
}
