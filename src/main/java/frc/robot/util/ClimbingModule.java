package frc.robot.util;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Climber.ClimberState;
import frc.robot.subsystems.Climber.ControlLoopType;

import static frc.robot.Constants.Climber.*;
import static frc.robot.Constants.*;

public class ClimbingModule implements Sendable{
    //Hardware
    private WPI_TalonFX motor; //Falcon that controls the height of the lift

    private ProfiledPIDController noLoadPID, loadPID, activePID;
    private SimpleMotorFeedforward noLoadFF, loadFF, activeFF;

    private ClimberState climberState = ClimberState.Lowered;
    private boolean braked = true;

    private ClimbingModule follower = null;
    private ClimbingModule leader = null;

    private boolean following = false; //This flag is here to ensure that a ClimbingModule that is already following cannot become the leader of anther motor

    private double pidOutput = 0;
    private double ffOutput = 0;
    private double climberTarget = 0; //The climber's current target in encoder counts    

    private String name;

    private boolean manualMode = false;

    public ClimbingModule(int motorID, PIDandFFConstants noLoadConstants, PIDandFFConstants loadConstants, String name) {
        motor = new WPI_TalonFX(motorID);

        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configSupplyCurrentLimit(CURRENT_LIMIT);

        noLoadPID = new ProfiledPIDController(noLoadConstants.getP(), noLoadConstants.getI(), noLoadConstants.getD(), 
        new TrapezoidProfile.Constraints(noLoadConstants.getMaxVel(), noLoadConstants.getMaxAccel()));
        noLoadFF = new SimpleMotorFeedforward(noLoadConstants.getKS(), noLoadConstants.getKV());

        loadPID = new ProfiledPIDController(loadConstants.getP(), loadConstants.getI(), loadConstants.getD(), 
        new TrapezoidProfile.Constraints(loadConstants.getMaxVel(), loadConstants.getMaxAccel()));
        loadFF = new SimpleMotorFeedforward(loadConstants.getKS(), loadConstants.getKV());

        activePID = noLoadPID;
        activeFF = noLoadFF;

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
    public void setModuleState(ClimberState state, ControlLoopType type, boolean overrideForceStop) {
        climberState = state;
        if(climberState == ClimberState.FullExtension) climberTarget = inchesToCounts(MID_HEIGHT);
        if(climberState == ClimberState.Low) climberTarget = inchesToCounts(LOW_HEIGHT);
        if(climberState == ClimberState.Lowered) climberTarget = inchesToCounts(LOWERED_HEIGHT);
        
        noLoadPID.reset(motor.getSelectedSensorPosition());
        loadPID.reset(motor.getSelectedSensorPosition());

        activePID = type == ControlLoopType.NoLoad ? noLoadPID : loadPID;
        activeFF = type == ControlLoopType.NoLoad ? noLoadFF : loadFF;
        
        if(braked && overrideForceStop) setBraked(false);

        if(follower != null) follower.setModuleState(state, type, overrideForceStop);
    }

    /**
     * Return a number of encoder counts based on the number of inches we want the climber to move.
     * @param inches the number of inches the climber should extend
     * @return encoder ticks 
     */
    public double inchesToCounts(double inches) {
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
     * Return a number of encoder counts based on the number of inches we want the climber to move.
     * @param counts the number of encoder counts
     * @return inches
     */
    private double countsToInches(double counts) {
        //Gear ratio details
        //Falcon shaft (8t) to gear 1 (54t)
        //Gear 1 (18t) to gear 2 (54t)
        //Gear 2 (54t) to pulley (1 in)

        return 
            counts              //The original 
            / 2048              //Encoder counts
            * (8.0 / 54.0)      //Number of revolutions (at motor shaft)
            * (18.0 / 54.0)     //Number of revolutions (at intermediate shaft)
            * Math.PI           //Number of revolutions (at output shaft)
        ;
    }

    /**
     * @return true if the climber is still outside of the acceptable error from the target (i.e. it's still moving.)
     */
    public boolean isBusy() {
        return Math.abs(motor.getSelectedSensorPosition() - climberTarget) > 5000;
    }

    /**
     * Method that should be periodically run in the Climber subsystem. It will also update the control loops of any followers
     */
    public void periodic() {

        pidOutput = activePID.calculate(motor.getSelectedSensorPosition(), climberTarget);
        ffOutput = activeFF.calculate(activePID.getSetpoint().velocity);
        double combinedOutput = pidOutput + ffOutput;

        //If any of these conditions are true, the motor should not be moving at all
        if(!(motor.getSelectedSensorPosition() < 0 && combinedOutput < 0 && !braked) && !manualMode) motor.setVoltage(combinedOutput);

        if((motor.getSelectedSensorPosition() <= 0 && combinedOutput < 0 && !manualMode) || braked) {
            motor.setVoltage(0);
        }
        
        if(follower != null) follower.periodic(); //Run the follower's control loop as well (if there is a follower set)

        SmartDashboard.putData("Active PID", activePID);
        SmartDashboard.putNumber("Active KV", activeFF.kv);
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

    public void reset() {motor.setSelectedSensorPosition(0);}

    private void setFollowing(boolean following) {this.following = following;}

    public boolean isFollowing() {return following;}
    public ClimberState getClimberState() {return climberState;}
    public double getPosition() {return motor.getSelectedSensorPosition();}
    public double getPositionInches() {return countsToInches(motor.getSelectedSensorPosition());}
    public double getVelocity() {return motor.getSelectedSensorVelocity() * 10;}
    public ProfiledPIDController getPID() {return noLoadPID;}
    public double getVoltage() {return pidOutput + ffOutput;}
    public String getName() {return name;}
    public void setLeader(ClimbingModule leader) {this.leader = leader;}
    public boolean isForceStopped() { return motor.getSensorCollection().isRevLimitSwitchClosed() == 1;}

    //Methods for moving the climber manually
    public void setManual(boolean value) { manualMode = value;}
    public void setMotors(double power) {motor.set(power);}

    //Methods for forcing the climbers to stop (i.e. soft stop)
    public void setBraked(boolean value) {
        //If the climber is currently force stopped and that is changing, then we want to change the target to the current angle
        if(braked && !value) climberTarget = motor.getSelectedSensorPosition();
        braked = value;
    }

    public ControlLoopType getActiveControlLoopType() {
        return activePID.equals(noLoadPID) ? ControlLoopType.NoLoad : ControlLoopType.Load;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Climbing Module");
        builder.addStringProperty("Module name", () -> getName(), null);
        builder.addStringProperty("Active PID type", () -> getActiveControlLoopType().name(), null);
        builder.addDoubleProperty("Limit switch activated", () -> motor.getSensorCollection().isRevLimitSwitchClosed(), null);
        builder.addBooleanProperty("Braked", () -> braked, null);
        builder.addDoubleProperty("Module target", () -> climberTarget, null);
        builder.addDoubleProperty("Measured position", () -> getPosition(), null);
        builder.addDoubleProperty("Target velocity", () -> activePID.getSetpoint().velocity, null);
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
        builder.addBooleanProperty("Busy", ()-> isBusy(), null);
    }


}
