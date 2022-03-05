package frc.robot.util;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.Climber.ClimberState;

import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.Constants.Climber.*;
import static frc.robot.Constants.*;

public class ClimbingModule implements Sendable{
    //Hardware
    private WPI_TalonFX motor; //Falcon that controls the height of the lift

    private ProfiledPIDController pidController;
    private SimpleMotorFeedforward ffController;

    private ClimberState climberState = ClimberState.Lowered;
    private boolean braked = true;

    private ClimbingModule follower = null;
    private ClimbingModule leader = null;

    private boolean following = false; //This flag is here to ensure that a ClimbingModule that is already following cannot become the leader of anther motor

    private double pidOutput = 0;
    private double ffOutput = 0;
    private double climberTarget = 0; //The climber's current target in encoder counts    
    //This value is set to true if one of the conditions is fulfilled that must reset the motor to zero.  It's purpose is to reset the pidController to avoid weird super high voltage spikes
    private boolean forceStop = false; 

    private String name;

    private boolean manualMode = false;

    public ClimbingModule(int motorID, PIDandFFConstants controllerConstants, String name) {
        motor = new WPI_TalonFX(motorID);

        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configSupplyCurrentLimit(CURRENT_LIMIT);

        pidController = new ProfiledPIDController(controllerConstants.getP(), controllerConstants.getI(), controllerConstants.getD(), 
        new TrapezoidProfile.Constraints(controllerConstants.getMaxVel(), controllerConstants.getMaxAccel()));
        ffController = new SimpleMotorFeedforward(controllerConstants.getKS(), controllerConstants.getKV());

        this.name = name;
    }

    /**
     * Method used to invert the motor on the module if necessary
     * @param invertType
     */
    public void setInverted(TalonFXInvertType invertType) {
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
            * (54.0 / 8.0)      //Number of revolutions (at motor shaft)
            * 2048              //Encoder counts
        ;
    }

    /**
     * @return true if the climber is still outside of the acceptable error from the target (i.e. it's still moving.)
     */
    public boolean isBusy() {
        return Math.abs(motor.getSelectedSensorPosition() - climberTarget) > 15000;
    }

    /**
     * Method that should be periodically run in the Climber subsystem. It will also update the control loops of any followers
     */
    public void periodic() {
        
        pidOutput = pidController.calculate(motor.getSelectedSensorPosition(), climberTarget);
        ffOutput = ffController.calculate(pidController.getSetpoint().velocity);
        double combinedOutput = pidOutput + ffOutput;

        //If any of these conditions are true, the motor should not be moving at all
        if(!braked && !(motor.getSelectedSensorPosition() < 0 && combinedOutput < 0) && !manualMode) motor.setVoltage(combinedOutput);

        if(motor.getSelectedSensorPosition() <= 0 && combinedOutput < 0 && !manualMode) motor.setVoltage(0);
        if(braked) motor.setVoltage(0);
        
        
        if(follower != null) follower.periodic(); //Run the follower's control loop as well (if there is a follower set)
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
            follower.setLeader(this);
            return true;
        } else {return false;}
    }

    public void reset() {
        motor.setSelectedSensorPosition(0);
        if(follower != null) follower.reset();
    }

    private void setFollowing(boolean following) {this.following = following;}

    public boolean isFollowing() {return following;}
    public boolean isBraked() {return braked;}
    public ClimberState getClimberState() {return climberState;}
    public double getPosition() {return motor.getSelectedSensorPosition();}
    public double getVelocity() {return motor.getSelectedSensorVelocity() * 10;}
    public ProfiledPIDController getPID() {return pidController;}
    public double getVoltage() {return pidOutput + ffOutput;}
    public String getName() {return name;}
    public void setLeader(ClimbingModule leader) {this.leader = leader;}
    public boolean isForceStopped() { return motor.getSensorCollection().isRevLimitSwitchClosed() == 1;}
    public void setBraked(boolean value) {braked = value;}

    public void setManual(boolean value) {
        manualMode = value;

        if(follower != null) follower.setManual(value);
    }

    public void setMotors(double power) {
        motor.set(power);

        if(follower != null) follower.setMotors(power);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Climbing Module");
        builder.addStringProperty("Module name", () -> getName(), null);
        builder.addDoubleProperty("Limit switch activated", () -> motor.getSensorCollection().isRevLimitSwitchClosed(), null);
        builder.addBooleanProperty("Force stopped", () -> forceStop, null);
        builder.addDoubleProperty("Module target", () -> climberTarget, null);
        builder.addDoubleProperty("Measured position", () -> getPosition(), null);
        builder.addDoubleProperty("Target velocity", () -> pidController.getSetpoint().velocity, null);
        builder.addDoubleProperty("Measured velocity", () -> getVelocity(), null);
        builder.addDoubleProperty("PID Output", () -> pidOutput, null);
        builder.addDoubleProperty("FF Output", () -> ffOutput, null);
        builder.addDoubleProperty("Motor voltage", () -> getVoltage(), null);
        builder.addBooleanProperty("Following another module?", () -> following, null);
        builder.addStringProperty("Name of leading module", () -> {
            return leader != null ? leader.getName() : "Not following another module";
        }, null);
        builder.addStringProperty("Climber state", () -> this.getClimberState().name(), null);
        builder.addDoubleProperty("PID target", () -> this.getPID().getSetpoint().position, null);
        builder.addBooleanProperty("Braked", () -> braked, null);
        builder.addBooleanProperty("Busy", ()-> isBusy(), null);
    }
}
