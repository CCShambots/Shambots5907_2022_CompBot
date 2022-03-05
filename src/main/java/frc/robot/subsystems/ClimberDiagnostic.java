package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Climber.*;
import static frc.robot.Constants.Drivetrain.*;


public class ClimberDiagnostic extends SubsystemBase{
    public WPI_TalonFX leftMotor = new WPI_TalonFX(LEFT_CLIMBER_ID);
    public WPI_TalonFX rightMotor = new WPI_TalonFX(RIGHT_CLIMBER_ID);

    public Solenoid brake = new Solenoid(COMPRESSOR_ID, PneumaticsModuleType.REVPH, BRAKE);

    public ClimberDiagnostic() {

        leftMotor.setInverted(TalonFXInvertType.CounterClockwise);
        leftMotor.setInverted(TalonFXInvertType.Clockwise);
    }

    public void setMotors(double power) {
        leftMotor.set(power);
        rightMotor.set(power);
    }
}
