package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.Drivetrain.*;

public class DrivetrainVelocityTuner extends CommandBase{
    private Drivetrain drivetrain;

    private double currentPercentage = 0;
    private double holdTime = 3000; //The time the drivetrain will hold at the maximum speed
    private double incrementPercentage = 0.01; //The time it takes (in MS) to go from zero to full speed

    private double holdStart = 0;

    private States state = States.Accelerate;

    public DrivetrainVelocityTuner(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override 
    public void execute() {
        if(state == States.Accelerate) {
            if(currentPercentage >= 1) {
                state = States.HoldForward;
                holdStart = System.currentTimeMillis();
                return;
            } 

            currentPercentage += incrementPercentage;

            setVelocity(currentPercentage);
        }
        else if(state == States.HoldForward) {
            if(System.currentTimeMillis() - holdStart >= holdTime) {
                state = States.Deccelerate;
            }
        }
        else if(state == States.Deccelerate) {
            if(currentPercentage <= -1) {
                state = States.HoldBackward;
                holdStart = System.currentTimeMillis();
                return;
            }

            currentPercentage -= incrementPercentage;
            
            setVelocity(currentPercentage);
        }
        else if(state == States.HoldBackward) {
            if(System.currentTimeMillis() - holdStart >= holdTime) {
                state = States.Accelerate;
            }
        }
    
    }

    private void setVelocity(double percentage) {
        drivetrain.tankDriveJoystick(percentage*MAX_LINEAR_VELOCITY, percentage*MAX_LINEAR_VELOCITY);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.tankDriveJoystick(0, 0);
    }

    private static enum States {
        Accelerate,
        HoldForward,
        Deccelerate,
        HoldBackward
    }
    
}
