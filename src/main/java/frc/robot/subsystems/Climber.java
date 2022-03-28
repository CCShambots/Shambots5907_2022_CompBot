package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants.Drivetrain;
import frc.robot.util.ClimbingModule;
import frc.robot.util.PIDandFFConstants;
import frc.robot.util.priorityFramework.PrioritizedSubsystem;

import static frc.robot.Constants.Climber.*;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

public class Climber extends PrioritizedSubsystem {
    //Important thing to remember when reading this: 
    //The right module follows the left module, meaning that any command sent through the left module also does that on the right module.

    private PIDandFFConstants noLoadConstants = new PIDandFFConstants(NO_LOAD_P, NO_LOAD_I, NO_LOAD_D, NO_LOAD_KS, NO_LOAD_KV, NO_LOAD_MAX_VEL, NO_LOAD_MAX_ACCEL);
    private PIDandFFConstants loadConstants = new PIDandFFConstants(LOAD_P, LOAD_I, LOAD_D, LOAD_KS, LOAD_KV, LOAD_MAX_VEL, LOAD_MAX_ACCEL);

    private ClimbingModule leftModule = new ClimbingModule(LEFT_CLIMBER_ID, noLoadConstants, loadConstants, "Left");
    private ClimbingModule rightModule = new ClimbingModule(RIGHT_CLIMBER_ID, noLoadConstants, loadConstants, "Right");

    private DoubleSolenoid solenoid = new DoubleSolenoid(Drivetrain.COMPRESSOR_ID, PneumaticsModuleType.CTREPCM, CLIMBER_PORT_1, CLIMBER_PORT_2);
    private Solenoid brake = new Solenoid(Drivetrain.COMPRESSOR_ID, PneumaticsModuleType.CTREPCM, BRAKE);

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

    public void setClimberState(ClimberState state, ControlLoopType type) {leftModule.setModuleState(state, type);}
    public boolean isUp() {return leftModule.getClimberState() != ClimberState.Lowered;} 
    public boolean isBusy() { return leftModule.isBusy() || rightModule.isBusy();}
    public boolean isForceStopped() {return leftModule.isForceStopped() || rightModule.isForceStopped();}
    public void setForceStopped(boolean value) {
        leftModule.setBraked(value);
        rightModule.setBraked(value);
    }

    public double getLeftPosition() {return leftModule.getPosition();}
    public double getRightPosition() {return rightModule.getPosition();}
    public double getLeftPositionInches() {return leftModule.getPositionInches();}
    public double getRightPositionInches() {return rightModule.getPositionInches();}
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
        rightModule.reset();
    }

    public void setSolenoids(Value v) {solenoid.set(v);}
    public Value getSolenoidState() {return solenoid.get();}

    /**
     * Command that will manually move one of the motors at a power and reset that motos's module when it finishes
     * @param power 0.0-1.0
     * @param side Left or right module to move
     * @return The command to run
     */
    public FunctionalCommand moveMotor(double power, MotorSide side) {

        ClimbingModule module = side == MotorSide.Right ? rightModule : leftModule;

        return new FunctionalCommand(() -> {
            unBrake();
            setManual(true);
            module.setMotors(power);
            System.out.println("Set a motor power");
          }, () -> {}, (interrupted) -> {
            module.setMotors(0);
            setManual(false);
            module.reset();
            brake();
          }, () -> false, this);
    }

    public FunctionalCommand moveMotors(double power) {

        return new FunctionalCommand(() -> {
            unBrake();
            setManual(true);
            leftModule.setMotors(power);
            rightModule.setMotors(power);
          }, () -> {}, (interrupted) -> {
            leftModule.setMotors(0);
            rightModule.setMotors(0);
            setManual(false);
            leftModule.reset();
            rightModule.reset();
            brake();
          }, () -> false, this);
    }

    /**
     * A command that does nothing and will end once the climber's measured position is greater than the given threshold
     * @param threshhold
     * @return the command to run
     */
    public FunctionalCommand waitForMovementCommand(double threshhold) {
        return new FunctionalCommand(() -> {}, () -> {}, (interrupted) -> {}, () -> getLeftPosition() > leftModule.inchesToCounts(threshhold) && getRightPosition() > leftModule.inchesToCounts(threshhold));
    }


    @Override
    public void periodic() {
        leftModule.periodic(); //Run the main control loop on the left module, which also updates the right module

        SmartDashboard.putData("Left module", leftModule);
        SmartDashboard.putData("right module", rightModule);
        SmartDashboard.putData("Solenoid state", solenoid);
    }
    
    public static enum ClimberState {Low, FullExtension, Lowered};
    public static enum ControlLoopType {NoLoad, Load};
    public static enum MotorSide {Left, Right};
}
