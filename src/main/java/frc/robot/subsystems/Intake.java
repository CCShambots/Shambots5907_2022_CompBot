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

  private DoubleSolenoid rotationalSolenoid = new DoubleSolenoid(COMPRESSOR_ID, PneumaticsModuleType.REVPH, 1, 2);

  private IntakeDirection direction = IntakeDirection.Stopped;
  private IntakeState intakeState = IntakeState.Raised;

  /** Creates a new Intake. */
  public Intake(){
    setupMotor(roller1);
    setupMotor(roller2);

    rotationalSolenoid.toggle();
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
    if(state == IntakeState.Raised && getRotationalPosition() == Value.kForward)  {
      stop();
      rotationalSolenoid.toggle();
    }
    else if(state == IntakeState.Lowered && getRotationalPosition() == Value.kReverse) { 
      rotationalSolenoid.toggle();

    }
  }

  public void raiseIntake() {setIntakeState(IntakeState.Raised);}
  public void lowerIntake() {setIntakeState(IntakeState.Lowered);}

  public Value getRotationalPosition() {
    return rotationalSolenoid.get();
  }

  public IntakeState getIntakeState() {
    return intakeState;
  }


  //Code for spinning the intake

  //TODO: Prioritize running the intake either forward or backward
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
    //TODO: Remove this once we figure out how to actually use solenoids
    SmartDashboard.putData(rotationalSolenoid);

    
  }

  public static enum IntakeState {
    Raised, Lowered
  }
  
  public enum IntakeDirection {
    Intaking, Exhausting, Stopped
}
}
