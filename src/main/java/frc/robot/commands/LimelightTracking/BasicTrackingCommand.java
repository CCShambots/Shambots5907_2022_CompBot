package frc.robot.commands.LimelightTracking;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Turret.Direction;

import static frc.robot.Constants.Turret.*;

/**
 * The basic implementation of tracking with the turret. All it will do is actively move the turret towards the limelight target
 */
public abstract class BasicTrackingCommand extends CommandBase{
    protected Turret turret;

    //Config values
    private LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);
    private double loopsBetweenSample = 7; //TODO: Tune this to the minimum value (fastest response time) that doesn't oscillate
    private double waitTime = 100; //Time (in ms) to wait before trusting the limelight value

    //Control variables (control the state of the command)
    private Mode mode = Mode.Targeting;
    private long startTimeMS = 0; //Time the command begins
    private double limelightOffset = 0; //The value of the target off on the limelight (x-axis)
    private double lowPassOutput = 0; //The current output of the SinglePoleIIR (basically low-pass) filter
    //Note: it starts at a ridiculously high value so the turret will immediately sample
    private double loopsSinceSample = 1000000; //Tracking variable for the number of loops since the last time the turret was commanded to move.
    private double targetAngle = 0; //The shooter's current setpoint
    private Direction prevDirection = Direction.Clockwise;

    /**Command to turn automatically towards the target using the limelight */
    public BasicTrackingCommand(Turret turret, Subsystem... requirements) {
        this.turret = turret;
        addRequirements(turret);

        for(Subsystem s : requirements) {
            addRequirements(s);
        }
    }

    /**Turns on the limelight, resets the gyro, and sets the limeLightOffset to whatever is currently on the network table */
    @Override
    public void initialize() {
        turret.setLimelightOn();
        startTimeMS = System.currentTimeMillis();

        //Code that runs in specific implementations
        additionalCodeInInitialize();
    }

    /** */
    @Override
    public void execute() {
        updateLimelight();
        
        //This if statement executes if the command has been running long enough that the limelight can be trusted
        if(System.currentTimeMillis() - startTimeMS >= waitTime) {
            if(mode == Mode.Targeting) targetingLoop();
            if(mode == Mode.Searching) searchingLoop();
        }

        additionalCodeInExecute();

        //TODO: Remove telemetry when no longer debugging
        SmartDashboard.putBoolean("Turret overextended?", turret.isOverRotated());
        SmartDashboard.putNumber("Rolling Average", lowPassOutput);
        SmartDashboard.putString("Mode", mode.name());
    }


    private void updateLimelight() {
        limelightOffset = turret.correctLimelightAngle(turret.getLimelightOffset()); //Get the limelight offset from the network table

        //Deadband the limelightOffset if it's within 2 degrees (to avoid oscillations)
        //TODO: tune this number for good deadzoning relative to accuracy
        if(Math.abs(limelightOffset) < 2) limelightOffset = 0;

        lowPassOutput = filter.calculate(limelightOffset);

        //Set the limelight value if the limelight has a target and enough loops have passed since the last time the shooter's angle was commanded
        if(turret.doesLimelightHaveTarget() && loopsSinceSample >= loopsBetweenSample) { 
            targetAngle =  lowPassOutput + turret.getSpinnerAngle();
            loopsSinceSample = 0;
        } else {
            loopsSinceSample++;
        }
    }

    private void targetingLoop() {
        if(turret.doesLimelightHaveTarget()) {
            //Set the spinner to the target angle
            turret.setSpinnerTarget(targetAngle);
        } else {
            //Searching for target
            mode = Mode.Searching;

            //Slow down the spinner's velocity for searching for the target
            turret.setSpinnerConstraints(new TrapezoidProfile.Constraints(SEARCH_VEL, SPINNER_MAX_ACCEL));
            setSearchTarget(turret.getSearchDirection());

            prevDirection = turret.getSearchDirection();
        }
    }

    private void searchingLoop() {

        //Update the direction the turret is moving if that has been indicated by the turret
        if(turret.getSearchDirection() != prevDirection) {
            setSearchTarget(turret.getSearchDirection());
        }
        prevDirection = turret.getSearchDirection();

        //End the searching loop if the limelight has a target
        if(turret.doesLimelightHaveTarget()) {
            mode = Mode.Targeting;  

            //Return the spinner to the original velocity constraints for live targeting of the limelight
            turret.setSpinnerConstraints(new TrapezoidProfile.Constraints(SPINNER_MAX_VEL, SPINNER_MAX_ACCEL));
            turret.setSpinnerTarget(turret.getSpinnerAngle());
        }
    }

    /**
     * Sets the spinner's target to the whatever side is indicated by the passed direction
    */
    private void setSearchTarget(Direction d) {
        if(d == Direction.Clockwise) turret.setSpinnerTarget(SPINNER_CLOCKWISE_LIMIT);
            else turret.setSpinnerTarget(SPINNER_COUNTERCLOCKWISE_LIMIT);

        System.out.print("Set search target to " + d.name());

    }

    /**
     * 
     * @return true if the turret is fully locked into the target (target visible, spinner fully moved, in targeting mode, limelight is trusted)
     */
    protected boolean isLockedIn() {
        return
            !turret.isSpinnerBusy() &&
            turret.doesLimelightHaveTarget() &&
            System.currentTimeMillis() - startTimeMS > waitTime &&
            mode == Mode.Targeting
        ;
    }


    /**
     * Turn off the limelight and make sure the spinner is set to it's current angle
     * */
    @Override
    public void end(boolean interrupted) {
        turret.setLimelightOff();
        turret.setSpinnerTarget(turret.getSpinnerAngle());
        turret.setFlywheelTarget(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }


    /* METHODS THAT SHOULD BE OVERWRITTEN FOR DIFFERENT BEHAVIORS IN DIFFERENT COMMANDS */
    //Is finished already taken >:(
    /**
     * @return whether the command is finished or not
     */
    public abstract boolean isComplete();
    /**
     * The behavior you want the flywheel to use to spinup (called in initialize)
     */
    public abstract void additionalCodeInInitialize();

    /**
     * Any extra code you want to run in execute()
     */
    public abstract void additionalCodeInExecute();

    /**
     * Any extra code that should be called in end
     */
    public abstract void additionalCodeInEnd();
    
    public static enum Mode {
        Targeting, Searching
    } 
}  
