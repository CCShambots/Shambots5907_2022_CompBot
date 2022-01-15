// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Intake extends SubsystemBase {

  
  private static final WPI_TalonFX cargoIntakeMotor = new WPI_TalonFX(Constants.CARGO_INTAKE_MOTOR);
  private static final WPI_TalonFX rotateIntakeMotor = new WPI_TalonFX(Constants.ROTATE_INTAKE_MOTOR);

  /** Creates a new Intake. */
  public Intake(){

  }
  
  public void runIntakeForward(){
  cargoIntakeMotor.set (0.5);
  }

  public void runIntakeBackward(){
    cargoIntakeMotor.set (-0.5);
  }
  public void stopIntake(){
    cargoIntakeMotor.set (0);
  }
  
  
  public void runRotateIntakeForward(){
  rotateIntakeMotor.set (0.5);
  }

  public void runRotateIntakeBackward(){
    rotateIntakeMotor.set (-0.5);
  }
  public void stopRotateIntake(){
    rotateIntakeMotor.set (0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
