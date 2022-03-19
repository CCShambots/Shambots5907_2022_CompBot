package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants.Drivetrain;
import frc.robot.util.ClimbingModule;
import frc.robot.util.PIDandFFConstants;
import frc.robot.util.priorityFramework.PrioritizedSubsystem;

import static frc.robot.Constants.Climber.*;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

public class Climber extends PrioritizedSubsystem {
    //Important thing to remember when reading this: The right module follows the left module, meaning that any command sent through the left module also does that on the right module.

    private ClimbingModule leftModule = new ClimbingModule(LEFT_CLIMBER_ID,
        new PIDandFFConstants(LEFT_P, LEFT_I, LEFT_D, LEFT_KS, LEFT_KV, MAX_VEL, MAX_ACCEL), "Left");

    private ClimbingModule rightModule = new ClimbingModule(RIGHT_CLIMBER_ID, 
        new PIDandFFConstants(RIGHT_P, RIGHT_I, RIGHT_D, RIGHT_KS, RIGHT_KV, MAX_VEL, MAX_ACCEL), "Right");

    private Solenoid brake = new Solenoid(Drivetrain.COMPRESSOR_ID, PneumaticsModuleType.REVPH, BRAKE);

    public Climber(){
        leftModule.lead(rightModule); //Setup the right module to follow the left module

        leftModule.setInverted(TalonFXInvertType.Clockwise);
        rightModule.setInverted(TalonFXInvertType.Clockwise);
    }   
    
    /**
     * Changies the physical state of the brake solenoids
     * @param braked
     */
    private void setSolenoids(boolean braked) {
        brake.set(!braked);
        leftModule.setBraked(braked);
        rightModule.setBraked(braked);
    }

    public void brake(){setSolenoids(true);}
    public void unBrake(){setSolenoids(false);}
    public void setClimberState(ClimberState state) {leftModule.setModuleState(state);}
    public boolean isUp() {return leftModule.getClimberState() != ClimberState.Lowered;} 
    public boolean isBusy() { return leftModule.isBusy() || rightModule.isBusy();}
    public boolean isForceStopped() {return leftModule.isForceStopped() || rightModule.isForceStopped();}

    public double getLeftPosition() {return leftModule.getPosition();}
    public double getRightPosition() {return rightModule.getPosition();}
    public double getLeftVelocity() {return leftModule.getVelocity();}
    public double getRightVelocity() {return rightModule.getVelocity();}
    public ProfiledPIDController getLeftPID() {return leftModule.getPID();}
    public ProfiledPIDController getRightPID() {return rightModule.getPID();}
    public double getLeftVoltage() {return leftModule.getVoltage();}
    public double getRightVoltage() {return rightModule.getVoltage();}

    public void setMotors(double power) {leftModule.setMotors(power); rightModule.setMotors(power);}
    public void setManual(boolean value) {leftModule.setManual(value); rightModule.setManual(value);}

    public void resetClimber() {
        leftModule.reset();
    }

    public FunctionalCommand moveMotor(double power, MotorSide side, boolean zero) {

        ClimbingModule module = side == MotorSide.Right ? rightModule : leftModule;

        return new FunctionalCommand(() -> {
            unBrake();
            setManual(true);
            module.setMotors(power);
          }, () -> {}, (interrupted) -> {
            module.setMotors(0);
            brake();
            setManual(false);
            module.reset();
          }, () -> false, this);
    }

    @Override
    public void periodic() {
        leftModule.periodic(); //Run the main control loop on the left module, which also updates the right module

        SmartDashboard.putData("Left module", leftModule);
        SmartDashboard.putData("right module", rightModule);
    }
    
    public static enum ClimberState { Low, Mid, Lowered};

    public static enum MotorSide { Left, Right};
}
