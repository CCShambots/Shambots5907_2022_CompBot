package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ClimbingModule;
import frc.robot.util.PIDandFFConstants;

import static frc.robot.Constants.Climber.*;

public class Climber extends SubsystemBase {
    //Important thing to remember when reading this: The right module follows the left module, meaning that any command sent through the left module also does that on the right module.

    private ClimbingModule leftModule = new ClimbingModule(LEFT_CLIMBER_ID, BRAKE_1_PORT_1, BRAKE_1_PORT_2, LEFT_LIMIT_SWITCH,
        new PIDandFFConstants(LEFT_P, LEFT_I, LEFT_D, LEFT_KS, LEFT_KV, MAX_VEL, MAX_ACCEL));

    private ClimbingModule rightModule = new ClimbingModule(RIGHT_CLIMBER_ID, BRAKE_2_PORT_1, BRAKE_2_PORT_2, RIGHT_LIMIT_SWITCH, 
        new PIDandFFConstants(RIGHT_P, RIGHT_I, RIGHT_D, RIGHT_KS, RIGHT_KV, MAX_VEL, MAX_ACCEL));

    public Climber(){
        leftModule.lead(rightModule); //Setup the right module to follow the left module

        //TODO: Reverse one of the modules if necessary
        // leftModule.setReversed(InvertType.OpposeMaster);

        leftModule.unBrake();
    }     

    public void brake(){leftModule.brake();}
    public void unBrake(){leftModule.unBrake();}
    public void setClimberState(ClimberState state) {leftModule.setModuleState(state);}
    public boolean isBusy() { return leftModule.isBusy() || rightModule.isBusy();}


    public double getLeftPosition() {return leftModule.getPosition();}
    public double getRightPosition() {return rightModule.getPosition();}
    public double getLeftVelocity() {return leftModule.getVelocity();}
    public double getRightVelocity() {return rightModule.getVelocity();}
    public ProfiledPIDController getLeftPID() {return leftModule.getPID();}
    public ProfiledPIDController getRightPID() {return rightModule.getPID();}
    public double getLeftVoltage() {return leftModule.getVoltage();}
    public double getRightVoltage() {return rightModule.getVoltage();}

    @Override
    public void periodic() {
        leftModule.periodic(); //Run the main control loop on the left module, which also updates the right module

        //TODO: Remove this telemetry once we're done with it
        //TODO: Write this telemtry before we use it
    }
    
    public static enum ClimberState { Low, Mid, Lowered};
}
