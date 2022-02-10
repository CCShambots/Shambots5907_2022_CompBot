package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private WPI_TalonFX talonOne;
    private WPI_TalonFX talonTwo;

    private Solenoid brake;

    public Climber(){
        talonOne = new WPI_TalonFX(Constants.Climber.TALON_ONE_PORT);
        talonTwo = new WPI_TalonFX(Constants.Climber.TALON_TWO_PORT);
        talonOne.follow(talonTwo);

        brake = new Solenoid(PneumaticsModuleType.REVPH, Constants.Climber.BRAKE_PORT);
    }

    public void brake(){
        brake.set(true);
    }

    public void unBrake(){
        brake.set(false);

    }

    public void moveClimberMaxHeight(){
        new RunCommand(() -> {
            setMotorOn();
        }, this)
        .withInterrupt(this::motorHitMax)
        .andThen(this::setMotorOff);
    }
    
    private boolean motorHitMax(){
        return (talonOne.getSelectedSensorPosition() >= Constants.Climber.MAX_ENCODER_HEIGHT);
    }

    private void setMotorOn(){
        talonOne.set(50);
    }

    private void setMotorOff(){
        talonOne.set(0);
    }

        
    
        
}
