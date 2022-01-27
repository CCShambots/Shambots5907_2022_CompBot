package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Conveyor.*;
import static frc.robot.Constants.*;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class Conveyor extends SubsystemBase{
    private static WPI_TalonFX conveyor1 = new WPI_TalonFX(CONVEYOR_1);
    
    private static List<WPI_TalonFX> conveyorMotors = new ArrayList<>();

    public Conveyor() {
        conveyorMotors.add(conveyor1);

        for(WPI_TalonFX motor : conveyorMotors) {
            motor.configFactoryDefault();
            motor.configSupplyCurrentLimit(CURRENT_LIMIT);
        } 
    }

    public void intakeAll() {
        setSpeed(DEFAULT_CONVEYOR_SPEED);
    }

    public void exhaustAll() {
        setSpeed(-DEFAULT_CONVEYOR_SPEED);
    }

    public void setSpeed(double speed) {
        for(WPI_TalonFX motor : conveyorMotors) {
            motor.set(ControlMode.PercentOutput, speed);
        }
    }

}
