package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.FakeGyro;
import frc.robot.util.hardware.HallEffectSensor;
import frc.robot.util.hardware.Limelight;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import static frc.robot.Constants.Turret.*;
import static frc.robot.Constants.*;

public class Turret extends SubsystemBase{

    //Hardware devices
    private WPI_TalonFX bottomFlywheel = new WPI_TalonFX(BOTTOM_FLYWHEEL);
    private WPI_TalonFX spinner = new WPI_TalonFX(TURRET_SPINNER);
    private Limelight limelight = new Limelight();

    private HallEffectSensor centerHallEffect = new HallEffectSensor(HALL_EFFECT_CENTER);

    private FakeGyro fakeGyro;

    // Flywheel controls
    private PIDController bottomFlywheelPID = new PIDController(BOTTOM_FLYWHEEL_P, BOTTOM_FLYWHEEL_I, BOTTOM_FLYWHEEL_D);
    private SimpleMotorFeedforward bottomFlywheelFeedforward = new SimpleMotorFeedforward(BOTTOM_FLYWHEEL_S, BOTTOM_FLYWHEEL_V);

    //Monitoring variables for telemetry
    private double bottomFlywheelPIDOutput = 0;
    private double bottomFlywheelFFOutput = 0;
    private double topFlywheelPIDOutput = 0;
    private double topFlywheelFFOutput = 0;

    // Spinner controls
    private ProfiledPIDController spinnerPIDController = new ProfiledPIDController(
        SPINNER_P, SPINNER_I, SPINNER_D, 
        new TrapezoidProfile.Constraints(
            SPINNER_MAX_VEL, SPINNER_MAX_ACCEL));


    private SimpleMotorFeedforward spinnerFeedForward = new SimpleMotorFeedforward(SPINNER_S, SPINNER_V);

    private boolean spinnerOverRotated = false;
    private Direction overRotatedDirection = Direction.CounterClockwise;
    
    private double spinnerPIDOutput = 0;
    private double spinnerFFOutput = 0;

    private double spinnerSetpoint = 0;

    private Direction searchDirection = Direction.Clockwise; //The direction the turret will spin in if no targets are spotted
    private boolean shouldShoot = false; //Whether or not the turret should shoot if it is able (while tracking)

    public Turret(ShuffleboardTab driveTab) {
        bottomFlywheel.configFactoryDefault();
        spinner.configFactoryDefault();

        bottomFlywheel.configSupplyCurrentLimit(FLYWHEEL_CURRENT_LIMIT);
        spinner.configSupplyCurrentLimit(CURRENT_LIMIT);
        
        bottomFlywheel.setNeutralMode(NeutralMode.Coast);
        spinner.setNeutralMode(NeutralMode.Brake);

        spinner.setInverted(true);

        fakeGyro = new FakeGyro(() -> getSpinnerAngle());

        initShuffleboard(driveTab);

    }
    
    private void initShuffleboard(ShuffleboardTab driveTab) {
        driveTab.add("Fake gyro", fakeGyro);
    }

    /* Flywheel methods */

    public void setFlywheelTarget(double RPM) {
        bottomFlywheelPID.setSetpoint(RPM);
    }

    public double getFlywheelTarget() {
        return bottomFlywheelPID.getSetpoint();
    }

    public double getBottomFlyWheelRPM() {return countsToRPM(bottomFlywheel.getSelectedSensorVelocity());}

    /**
     * Converts a value, in encoder counts, to RPM
     * @param counts
     * @return
     */
    private double countsToRPM(double counts) {
        return counts * 10 / 2048.0 * 60;
    }

    /**
     * Gets whether the flywheel is still accelertaing towards it's target RPM
     * @return true if the flywheel is out of the allowed error (i.e. it's fat from the target RPM)
     */
    public boolean isFlywheelBusy() {
        return Math.abs(getBottomFlyWheelRPM() - getFlywheelTarget()) > FLYWHEEL_ALLOWED_ERROR;
    }

    public double getBottomFlywheelVoltage() {return bottomFlywheel.getMotorOutputVoltage();}
    public double getTopFlywheelVoltage() {return bottomFlywheel.getMotorOutputVoltage();}

    public double getBottomFlywheelPIDOutput() {return bottomFlywheelPIDOutput;}
    public double getBottomFlywheelFFOutput() {return bottomFlywheelFFOutput;}
    public double getTopFlywheelPIDOutput() {return topFlywheelPIDOutput;}
    public double getTopFlywheelFFOutput() {return topFlywheelFFOutput;}

    public boolean getShouldShoot() {return shouldShoot;}
    public void setShouldShoot(boolean value) {shouldShoot = value;}


    /* Spinner methods */

    /**
     * Sets the target for the spinner with protection for trying to over-extend the shooter.
     * If the shooter is "over-extended" a flag will be set for easy access
     * @param degrees The target angle of the spinner (positive going counter-clockwise)
     */
    public void setSpinnerTarget(double angle) {
        if(spinnerOverRotated) spinnerOverRotated = false;

        if(angle > SPINNER_COUNTERCLOCKWISE_LIMIT) {
            angle = SPINNER_COUNTERCLOCKWISE_LIMIT;
            spinnerOverRotated = true;
        } else if(angle < SPINNER_CLOCKWISE_LIMIT) {
            angle = SPINNER_CLOCKWISE_LIMIT;
            spinnerOverRotated = true;
        } 

        spinnerSetpoint = angle;
    }

    /**
     * Resets the spinner's PID (as it increases voltage when idling which can cause jerking if not reset)
     */
    public void resetSpinnerPID() {
        spinnerPIDController.reset(getSpinnerAngle());
        spinnerSetpoint = getSpinnerAngle();
    }

    public void setSpinnerNeutralMode(NeutralMode mode) {spinner.setNeutralMode(mode);}

    /**
     * @return Spinner target (in degrees)
     */
    public double getSpinnerTarget() {
        return spinnerSetpoint;
    }
    
    public boolean isOverRotated() {return spinnerOverRotated;}

    /**
     * @return returns the direction in which the turret has been over-extended (clockwise/counterclockwise)
     */
    public Direction getOverRotatedDirection() {
        return overRotatedDirection;
    }

    /**
     * @return true if the spinner is greater than ACCEPTABLE_ERROR degrees off from the target
     */
    public boolean isSpinnerBusy() {
        return Math.abs(getSpinnerAngle() - getSpinnerTarget()) >= ACCEPTABLE_ERROR;
    }  

    /**
     * @return true if the spinner is not in the range where it is unable to shoot
     */
    public boolean isShootingAllowed() {
        return !INVALID_SHOOTING_RANGE.inRange(getSpinnerAngle()) && !isFlywheelBusy();
    }

    /**
     * Returns the turrets angle, assuming zero is facing forwards
     * @return The turret's angle in degrees
     */
    public double getSpinnerAngle() {
        return 
            spinner.getSelectedSensorPosition() // Counts
            / Constants.Turret.COUNTS_SPINNER_ENCODER // Rotations
            * 360 //Degrees
            * Constants.Turret.TURRET_GEAR_RATIO //Degrees on the turret
            // * -1 //Convert to the right sign
        ;
    }

    /**
     * @param counts the desired encoder counds to reset the spinner to
     */
    private void resetSpinnerEncoder(double counts) {
        spinner.setSelectedSensorPosition(counts);
    }

    /**
     * @param angle the angle the spinner will be reset to
     */
    public void resetSpinnerAngle(double angle) {
        resetSpinnerEncoder(
            angle //degress
            // * -1 //Negate the encoder
            / Constants.Turret.TURRET_GEAR_RATIO //Degrees on the encoder
            / 360 //Rotations
            * Constants.Turret.COUNTS_SPINNER_ENCODER //Counts
        );

    }

    /**
     * @param constraints Method to update the constraints (max vel & max accel) on the spinner
     */
    public void setSpinnerConstraints(Constraints constraints) {
        spinnerPIDController.setConstraints(constraints);
    }

    public double getSpinnerPIDOutput() {return spinnerPIDOutput;}
    public double getSpinnerFeedForwardOutput() {return spinnerFFOutput;}
    public double getSpinnerVoltage() {return spinner.getMotorOutputVoltage();}
    public double getSpinnerTargetVelocity() {return spinnerPIDController.getSetpoint().velocity;}

    public double getSpinnerVelocity() {
        return 
            spinner.getSelectedSensorVelocity() //Counts per 100 ms
            * 10 //Counts per second (TalonFX is weird)
            / COUNTS_SPINNER_ENCODER //Rotations per second
            * 360 //Degrees per second (at motor)
            * TURRET_GEAR_RATIO //Degrees per second (on turret)
            // * -1 //Get the ratio correct for the direction the turret is moving
        ;
    }

    public Direction getSearchDirection() {return searchDirection;}
    public void setSearchDirection(Direction direction) {searchDirection = direction;}
    public void toggleSearchDirection() { 
        if(searchDirection == Direction.Clockwise) setSearchDirection(Direction.CounterClockwise);
        else setSearchDirection(Direction.Clockwise);
    }

    /* Other sensors */
    public boolean isHallEffectActivated() { return centerHallEffect.isActivated();}


    //Limelight meothods (just exposes the functions in the util class)
    public void setLimelightOn() {limelight.setOn();}
    public void setLimelightOff() {limelight.setOff();}
    public boolean doesLimelightHaveTarget() {return limelight.hasTarget();}
    public Translation2d getLimelightOffset() {return limelight.targetOffset();}
    public double getLimelightLatency() {return limelight.getLatency();}

    /**
     * 
     * @param offset
     * @return
     */
    public double correctLimelightAngle(Translation2d offset) {
        return -offset.getX();
    }

    @Override
    public void periodic() {
        bottomFlywheelPIDOutput = bottomFlywheelPID.calculate(getBottomFlyWheelRPM());
        bottomFlywheelFFOutput = bottomFlywheelFeedforward.calculate(bottomFlywheelPID.getSetpoint());

        if(bottomFlywheelPID.getSetpoint() == 0) bottomFlywheel.setVoltage(0);
        else bottomFlywheel.setVoltage(bottomFlywheelPIDOutput + bottomFlywheelFFOutput);
    

        spinnerPIDOutput = spinnerPIDController.calculate(getSpinnerAngle(), spinnerSetpoint);
        spinnerFFOutput = spinnerFeedForward.calculate(spinnerPIDController.getSetpoint().velocity);

        spinner.setVoltage(spinnerPIDOutput + spinnerFFOutput);


        //TODO: Remove this telemetry when it's no longer used
        
        //Flywheel telemetry
        SmartDashboard.putData("Bottom Flywheel PID", bottomFlywheelPID);
        SmartDashboard.putNumber("Top Flywheel PID Output", topFlywheelPIDOutput);
        SmartDashboard.putNumber("Bottom Flywheel PID Output", bottomFlywheelPIDOutput);
        SmartDashboard.putNumber("Top Flywheel FF Output", bottomFlywheelFFOutput);
        SmartDashboard.putNumber("Bottom Flywheel FF Output", bottomFlywheelFFOutput);
        SmartDashboard.putNumber("Bottom Flywheel Target Velo", bottomFlywheelPID.getSetpoint());
        SmartDashboard.putNumber("Bottom Flywheel Measured Velo", getBottomFlyWheelRPM());
        SmartDashboard.putNumber("Bottom Flywheel Voltage", bottomFlywheel.getMotorOutputVoltage());

        //Spinner telemetry
        SmartDashboard.putData("Spinner PID", spinnerPIDController);
        SmartDashboard.putNumber("Spinner PID Output", spinnerPIDOutput);
        SmartDashboard.putNumber("Spinner FF Output", spinnerFFOutput);
        SmartDashboard.putNumber("Spinner Target Velo", getSpinnerTargetVelocity());
        SmartDashboard.putNumber("Spinner Measured velo", getSpinnerVelocity());
        SmartDashboard.putNumber("Spinner Target Angle", getSpinnerTarget());
        SmartDashboard.putNumber("Spinner Measured Angle", getSpinnerAngle());
        SmartDashboard.putNumber("Spinner Voltage", getSpinnerVoltage());

        SmartDashboard.putBoolean("Is spinner busy", isSpinnerBusy());

    }

    public enum Direction {
        Clockwise, 
        CounterClockwise
    }


}
