package frc.robot.util;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.Climber.ClimberState;
import frc.robot.util.hardware.HallEffectSensor;

import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.Constants.Climber.*;
import static frc.robot.Constants.*;

public class ClimbingModule {

    //Hardware
    private WPI_TalonFX motor; //Falcon that controls the height of the lift
    private DoubleSolenoid brake; //Solenoid that activates the brake on the lift
    private HallEffectSensor limitSwitch; //Limit switch located at the bottom of the lift

    private ProfiledPIDController pidController;
    private SimpleMotorFeedforward ffController;

    private ClimberState climberState = ClimberState.Lowered;
    private boolean braked = false;
    private ClimbingModule follower = null;
    private boolean following = false; //This flag is here to ensure that a ClimbingModule that is already following cannot become the leader of anther motor

    private double pidOutput = 0;
    private double ffOutput = 0;
    private double climberTarget = 0; //The climber's current target in encoder counts    
    //This value is set to true if one of the conditions is fulfilled that must reset the motor to zero.  It's purpose is to reset the pidController to avoid weird super high voltage spikes
    private boolean forceStop = false; 

    public ClimbingModule(int motorID, int brake1Port, int brake2Port, int limitSwitchPin, PIDandFFConstants controllerConstants) {
        motor = new WPI_TalonFX(motorID);
        brake = new DoubleSolenoid(COMPRESSOR_ID, PneumaticsModuleType.REVPH, brake1Port, brake2Port);

        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configSupplyCurrentLimit(CURRENT_LIMIT);

        pidController = new ProfiledPIDController(controllerConstants.getP(), controllerConstants.getI(), controllerConstants.getD(), 
        new TrapezoidProfile.Constraints(controllerConstants.getMaxVel(), controllerConstants.getMaxAccel()));
        ffController = new SimpleMotorFeedforward(controllerConstants.getKS(), controllerConstants.getKV());

        limitSwitch = new HallEffectSensor(limitSwitchPin);
    }

    /**
     * Method used to invert the motor on the module if necessary
     * @param invertType
     */
    public void setReversed(InvertType invertType) {
        motor.setInverted(invertType);
    }

    /**
     * Update the target of the climbing module
     * @param state The location the climber should move to
     */
    public void setModuleState(ClimberState state) {
        climberState = state;
        if(climberState == ClimberState.Mid) climberTarget = inchesToCounts(MID_HEIGHT);
        if(climberState == ClimberState.Low) climberTarget = inchesToCounts(LOW_HEIGHT);
        if(climberState == ClimberState.Lowered) climberTarget = inchesToCounts(LOWERED_HEIGHT);

        if(follower != null) follower.setModuleState(state);
    }

    /**
     * Return a number of encoder counts based on the number of inches we want the climber to move.
     * @param inches the number of inches the climber should extend
     * @return encoder ticks 
     */
    private double inchesToCounts(double inches) {
        //Gear ratio details
        //Falcon shaft (8t) to gear 1 (54t)
        //Gear 1 (18t) to gear 2 (54t)
        //Gear 2 (54t) to pulley (1 in)

        return 
            inches              //The original 
            / Math.PI           //Number of revolutions (at output shaft)
            * (54.0 / 18.0)     //Number of revolutions (at intermediate shaft)
            * (54.0 / 9.0)      //Number of revolutions (at motor shaft)
            * 2048              //Encoder counts
        ;
    }

    /**
     * @return true if the climber is still outside of the acceptable error from the target (i.e. it's still moving.)
     */
    public boolean isBusy() {
        return Math.abs(motor.getSelectedSensorPosition() - climberTarget) > 5;
    }

    /**
     * Method that should be periodically run in the Climber subsystem. It will also update the control loops of any followers
     */
    public void periodic() {

        boolean pushingSwitch = pidOutput + ffOutput < 0 && limitSwitch.isActivated(); //Whether the motor is trying to move down while the limit switch is pressed
        boolean motorOverExtended = motor.getSelectedSensorPosition() >= MID_HEIGHT && pidOutput + ffOutput > 0; //Whether to motor has reached the top and it's trying to move up
        
        //If any of these conditions are true, the motor should not be moving at all
        if(pushingSwitch || motorOverExtended || isBraked()) {
            motor.setVoltage(0);
            forceStop = true;
        } else {
            //Reset the PID controller if the motor was just being forced to stop (as it can build up very high voltage setpoints when voltages are not actually applied)
            if(forceStop) {
                forceStop = false;
                pidController.reset(motor.getSelectedSensorPosition());
            }

            pidOutput = pidController.calculate(motor.getSelectedSensorPosition(), climberTarget);
            ffOutput = ffController.calculate(pidController.getGoal().velocity);

            motor.setVoltage(pidOutput + ffOutput);
        }


        if(follower != null) follower.periodic(); //Run the follower's control loop as well (if there is a follower set)
    }


    /**
     * Changies the physical state of the brake solenoids
     * @param braked
     */
    private void setSolenoids(boolean braked) {
        this.braked = braked;
        Value value = braked == true ? Value.kForward : Value.kReverse; 
        brake.set(value);

        if(follower != null) follower.setSolenoids(braked);
    }

    /**
     * Sets a ClimbingModule to follow the current instance of ClimbingModule
     * @param follower the module that should begin following
     * @return
     */
    public boolean lead(ClimbingModule follower) {
        if(!following) {
            this.follower = follower;
            follower.setFollowing(true);
            return true;
        } else {return false;}
    }

    private void setFollowing(boolean following) {this.following = following;}

    public void brake(){setSolenoids(true);;}
    public void unBrake(){setSolenoids(false);}

    public boolean isFollowing() {return following;}
    public boolean isBraked() {return braked;}
    public ClimberState getClimberState() {return climberState;}
    public double getPosition() {return motor.getSelectedSensorPosition();}
    public double getVelocity() {return getPosition() * 10;}
    public ProfiledPIDController getPID() {return pidController;}
    public double getVoltage() {return pidOutput + ffOutput;}
}
