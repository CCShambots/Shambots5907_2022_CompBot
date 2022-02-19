// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Intake.*;

import static frc.robot.Constants.*;
import static frc.robot.Constants.Drivetrain.*;


public class Intake extends SubsystemBase {
  private WPI_TalonFX roller1 = new WPI_TalonFX(ROLLER_1_ID);
  private WPI_TalonFX roller2 = new WPI_TalonFX(ROLLER_2_ID);

  private DoubleSolenoid rotationalSolenoid1 = new DoubleSolenoid(COMPRESSOR_ID, PneumaticsModuleType.REVPH, SOLENOID_1_PORT_1, SOLENOID_1_PORT_2);
  private DoubleSolenoid rotationalSolenoid2 = new DoubleSolenoid(COMPRESSOR_ID, PneumaticsModuleType.REVPH, SOLENOID_2_PORT_1, SOLENOID_2_PORT_2);

  private IntakeDirection direction = IntakeDirection.Stopped;
  private IntakeState intakeState = IntakeState.Raised;

  /** Creates a new Intake. */
  public Intake(){
    setupMotor(roller1);
    setupMotor(roller2);

    setIntakeState(IntakeState.Raised);
  }

  private void setupMotor(WPI_TalonFX motor) {
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configSupplyCurrentLimit(CURRENT_LIMIT);
  }


  //Code for rotating the intake up and down

    /**
   * Sets the target point of the intake's PID controller to the inputted state.
   * @param state Raised or lowered 
   */
  private void setIntakeState(IntakeState state) {
    intakeState = state;
    Value value = state == IntakeState.Raised ? Value.kForward : Value.kReverse;

    rotationalSolenoid1.set(value);
    rotationalSolenoid2.set(value);
  }

  public void raiseIntake() {setIntakeState(IntakeState.Raised);}
  public void lowerIntake() {setIntakeState(IntakeState.Lowered);}

  public Value getRotationalPosition() {
    return rotationalSolenoid1.get();
  }

  public IntakeState getIntakeState() {
    return intakeState;
  }


  //Code for spinning the intake
  public void intake(){setIntakeDirection(IntakeDirection.Intaking);}
  public void exhaust(){setIntakeDirection(IntakeDirection.Exhausting);}
  public void stop(){setIntakeDirection(IntakeDirection.Stopped);}

  /**
   * 
   * @param direction The direction the intake roller should move
   * @return Whether the motors were sucessfully set or not
   */
  private void setIntakeDirection(IntakeDirection direction) {
    this.direction = direction;
    if(direction == IntakeDirection.Intaking) {
      setMotors(INTAKE_SPEED);
    }else if(direction == IntakeDirection.Exhausting) {
      setMotors(-INTAKE_SPEED);
    } else {
      setMotors(0);
    }
  }

  public boolean isRunning() {return direction != IntakeDirection.Stopped;}

  private void setMotors(double speed) {
    roller1.set(ControlMode.PercentOutput, speed);
    roller2.set(ControlMode.PercentOutput, speed);
  } 

  public IntakeDirection getIntakeDirection() {
    return direction;
  }

  /**
   * Periodic loop
   */
  @Override
  public void periodic() {
    //TODO: Remove this telemetry after finished
    SmartDashboard.putData(rotationalSolenoid1);
    SmartDashboard.putData(rotationalSolenoid2);
    
  }

  public static enum IntakeState {
    Raised, Lowered
  }
  
  public enum IntakeDirection {
    Intaking, Exhausting, Stopped
}
}
