// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Intake.*;

import static frc.robot.Constants.*;


public class Intake extends SubsystemBase {
  //TODO: Add override for stopping control of the intake when it is up (and make it stop by default when up)

  private WPI_TalonFX cargoIntakeMotor = new WPI_TalonFX(CARGO_INTAKE_MOTOR_ID);

  private Compressor compressor = new Compressor(COMPRESSOR_ID, PneumaticsModuleType.REVPH);
  private DoubleSolenoid rotationalSolenoid = new DoubleSolenoid(COMPRESSOR_ID, PneumaticsModuleType.CTREPCM, 1, 2);

  private IntakeDirection direction = IntakeDirection.Stopped;
  private IntakeState intakeState = IntakeState.Raised;


  /** Creates a new Intake. */
  public Intake(){
    cargoIntakeMotor.configFactoryDefault();
    cargoIntakeMotor.setNeutralMode(NeutralMode.Brake);
    cargoIntakeMotor.configSupplyCurrentLimit(CURRENT_LIMIT);

    compressor.enableDigital();
    rotationalSolenoid.toggle();
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
    else if(state == IntakeState.Lowered && getRotationalPosition() == Value.kReverse) rotationalSolenoid.toggle();
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

  private void setIntakeDirection(IntakeDirection direction) {
    //The intake will not be allowed to be commanded to different speeds when it is raised
    if(getIntakeState() == IntakeState.Raised) return;

    this.direction = direction;
    if(direction == IntakeDirection.Intaking) {
      cargoIntakeMotor.set(INTAKE_SPEED);
    }else if(direction == IntakeDirection.Exhausting) {
      cargoIntakeMotor.set(-INTAKE_SPEED);
    } else {
      cargoIntakeMotor.set(0);
    }

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
