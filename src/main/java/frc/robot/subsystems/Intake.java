// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.IntakeDirections;
import frc.robot.util.IntakeStates;

import static frc.robot.util.IntakeDirections.*;
import static frc.robot.util.IntakeStates.*;
import static frc.robot.Constants.Intake.*;
import static frc.robot.Constants.*;


public class Intake extends SubsystemBase {

  //TODO: Add override for stopping control of the intake until it is fully in one state (up/down)

  private final WPI_TalonFX cargoIntakeMotor = new WPI_TalonFX(CARGO_INTAKE_MOTOR);
  private final WPI_TalonFX rotateIntakeMotor = new WPI_TalonFX(ROTATE_INTAKE_MOTOR);

  private IntakeDirections direction = Stopped;
  private IntakeStates intakeState = IntakeStates.Raised;

  private ProfiledPIDController rotationalPID = new ProfiledPIDController(ROTATIONAL_P, ROTATIONAL_I, ROTATIONAL_D, 
    new TrapezoidProfile.Constraints(ROTATIONAL_MAX_VEL, ROTATIONAL_MAX_ACCEL));
  private SimpleMotorFeedforward rotationalFeedForward = new SimpleMotorFeedforward(ROTATIONAL_KS, ROTATIONAL_KV);

  private double rotationalSetpoint = 0;
  private double rotationalPIDOutput = 0;
  private double rotationalFeedForwardOutput = 0;

  /** Creates a new Intake. */
  public Intake(){
    cargoIntakeMotor.configFactoryDefault();
    rotateIntakeMotor.configFactoryDefault();

    cargoIntakeMotor.setNeutralMode(NeutralMode.Brake);
    rotateIntakeMotor.setNeutralMode(NeutralMode.Brake);

    cargoIntakeMotor.configSupplyCurrentLimit(CURRENT_LIMIT);
    rotateIntakeMotor.configSupplyCurrentLimit(CURRENT_LIMIT);
  }


  //Code for rotating the intake up and down

    /**
   * Sets the target point of the intake's PID controller to the inputted state.
   * @param state Raised or lowered 
   */
  private void setTargetPoint(IntakeStates state) {
    intakeState = state;
    if(state == Raised) rotationalSetpoint = INTAKE_RAISED_COUNTS;
    else rotationalSetpoint = INTAKE_LOWERED_COUNTS;
  }

  public void raiseIntake() {
    setTargetPoint(Raised);
  }

  public void lowerIntake() {
    setTargetPoint(Lowered);
  }

  /**
   * Resets the encoder to the zero position (raised)
   */
  public void resetRotationalEncoder() {
    rotateIntakeMotor.setSelectedSensorPosition(0);
  }

  /**
   * @return The rotational intake encoder's position
   */
  public double getRotationalEncoder() {
    return rotateIntakeMotor.getSelectedSensorPosition();
  }

  public void rotateIntake(double power) {
    rotateIntakeMotor.set(power);
  }

  public IntakeStates getIntakeState() {
    return intakeState;
  }


  //Code for spinning the intake

  //TODO: Prioritize running the intake either forward or backward
  public void intakeBalls(){
    cargoIntakeMotor.set(INTAKE_SPEED);
    direction = Forwards;
  }

  public void exhaustBalls(){
    cargoIntakeMotor.set(INTAKE_SPEED);
    direction = Backwards;
  }

  public void stopIntake(){
    cargoIntakeMotor.set(0);
    direction = Stopped;
  }

  public IntakeDirections getIntakeMode() {
    return direction;
  }

  /**
   * Periodic loop
   */
  @Override
  public void periodic() {
    
    rotationalPIDOutput = rotationalPID.calculate(getRotationalEncoder(), rotationalSetpoint);
    rotationalFeedForwardOutput = rotationalFeedForward.calculate(rotationalPID.getGoal().velocity);

    rotateIntakeMotor.setVoltage(rotationalFeedForwardOutput + rotationalPIDOutput);
  }
}
