package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.FakeGyro;
import frc.robot.util.hardware.HallEffectSensor;
import frc.robot.util.hardware.LidarSensor;
import frc.robot.util.hardware.Limelight;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import static frc.robot.Constants.Turret.*;

import static frc.robot.Constants.Lidar.*;

public class Turret extends SubsystemBase{

    //Hardware devices
    private WPI_TalonFX flywheel = new WPI_TalonFX(FLYWHEEL);
    private WPI_TalonFX spinner = new WPI_TalonFX(TURRET_SPINNER);
    private Limelight limelight = new Limelight();

    private LidarSensor lidar = new LidarSensor(MONITOR_PIN, TRIGGER_PIN);
    private HallEffectSensor centerHallEffect = new HallEffectSensor(HALL_EFFECT_CENTER);

    private FakeGyro fakeGyro;
    private WPI_PigeonIMU gyro; 

    // Flywheel controls
    //TODO: Make private
    private PIDController flywheelPID = new PIDController(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D);
    private SimpleMotorFeedforward flywheelFeedForward = new SimpleMotorFeedforward(FLYWHEEL_S, FLYWHEEL_V);

    //Monitoring variables for telemetry
    private double flywheelPIDOutput = 0;
    private double flywheelFeedForwardOutput = 0;

    // Spinner controls
    private ProfiledPIDController spinnerPIDController = new ProfiledPIDController(
        SPINNER_P, SPINNER_I, SPINNER_D, 
        new TrapezoidProfile.Constraints(
            SPINNER_MAX_VEL, SPINNER_MAX_ACCEL));


    private SimpleMotorFeedforward spinnerFeedForward = new SimpleMotorFeedforward(SPINNER_S, SPINNER_V);

    private boolean spinnerOverRotated = false;
    private Direction overRotatedDirection = Direction.CounterClockwise;
    
    private double spinnerPIDOutput = 0;
    private double spinnerFeedForwardOutput = 0;

    private double spinnerSetpoint = 0;

    private Direction searchDirection = Direction.Clockwise; //The direction the turret will spin in if no targets are spotted

    public Turret(ShuffleboardTab driveTab) {
        flywheel.configFactoryDefault();
        spinner.configFactoryDefault();

        flywheel.configSupplyCurrentLimit(CURRENT_LIMIT);
        spinner.configSupplyCurrentLimit(CURRENT_LIMIT);

        spinner.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        
        flywheel.setNeutralMode(NeutralMode.Coast);
        spinner.setNeutralMode(NeutralMode.Brake);

        fakeGyro = new FakeGyro(() -> 5);
        gyro = new WPI_PigeonIMU(0);

        initShuffleboard(driveTab);

    }

    private void initShuffleboard(ShuffleboardTab driveTrab) {
        // driveTrab.add("Spinner Angle", fakeGyro)
        // .withWidget(BuiltInWidgets.kGyro);
        // // .withProperties(Map.of("min", SPINNER_COUNTERCLOCKWISE_LIMIT, "max", SPINNER_CLOCKWISE_LIMIT))
        // ;

        gyro.setFusedHeading(1000000000);

        driveTrab.add("Analog Angle", gyro)
        .withWidget(BuiltInWidgets.kGyro);
        // .withProperties(Map.of("min", SPINNER_COUNTERCLOCKWISE_LIMIT, "max", SPINNER_CLOCKWISE_LIMIT))
        ;

        driveTrab.addNumber("Spinner angle (just number)", () -> getSpinnerAngle());
    }

    /* Flywheel methods */
    
    /**
     * Given a distance, it sets the flywheel to the correct RPM
     * @param distance The distance away from the target
     */
    public void spinUpFlywheel(double distance) {
        //TODO: Calculate speed based on distance
        double rpm = 6000;

        setFlywheelTarget(rpm);
    }

    /**
     * @param RPM The flywheel's target (in RPM)
     */
    public void setFlywheelTarget(double RPM) {
        flywheelPID.setSetpoint(RPM);
    }

    /**
     * @return The flywhee's target RPM
     */
    public double getFlywheelTarget() {
        return flywheelPID.getSetpoint();
    }

    /**
     * @return The measured RPM of the flywheel
     */
    public double getFlyWheelRPM() {
        return flywheel.getSelectedSensorVelocity() 
        * 10 //To Counts/sec
        / 2048 //To rotations/sec
        * 60 //To rotations per minute
        ;
    }

    /**
     * Gets whether the flywheel is still accelertaing towards it's target RPM
     * @return true if the flywheel is within the allowed error (i.e. it's close the target RPM); false if still not in that range
     */
    public boolean isFlywheelBusy() {
        return Math.abs(getFlywheelTarget() - getFlywheelTarget()) > FLYWHEEL_ALLOWED_ERROR;
    }

    /**
     * Telemtry function
     * @return the voltage the flywheel has been set to
     */
    public double getFlywheelVoltage() {
        return flywheel.getMotorOutputVoltage();
    }

    /**
     * Resets the spinner's PID (as it increases voltage when idling which can cause jerking if not reset)
     */
    public void resetSpinnerPID() {
        spinnerPIDController.reset(getSpinnerAngle());
        spinnerSetpoint = getSpinnerAngle();
    }

    /**
     * @return The current PID output of the flywheel
     */
    public double getFlywheelBangBangOutput() {
        return flywheelPIDOutput;
    }

    /**
     * @return The current Feedforward output of the flywheel
     */
    public double getFlywheelFeedforwardOutput() {
        return flywheelFeedForwardOutput;
    }


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
     * @return The target angle of the spinner (in degrees)
     */
    public double getSpinnerTarget() {
        return spinnerSetpoint;
    }
    
    /**
     * @return returns true if the turret was commanded to go farther than it can
     */
    public boolean isOverRotated() {
        return spinnerOverRotated;
    }

    /**
     * @return returns the direction in which the turret has been over-extended (clockwise/counterclockwise)
     */
    public Direction getOverRotatedDirection() {
        return overRotatedDirection;
    }

    /**
     * @return true if the spinner is less than ACCEPTABLE_ERROR degrees off from the target
     */
    public boolean isSpinnerBusy() {
        return Math.abs(getSpinnerAngle() - getSpinnerTarget()) <= ACCEPTABLE_ERROR;
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
            /Constants.Turret.COUNTS_SPINNER_ENCODER // Rotations
            * 360 //Degrees
            * Constants.Turret.TURRET_GEAR_RATIO //Degrees on the turret
            * -1 //Convert to the right angle
        ;
    }

    /**
     * @return The encoder value of the spinner (positive going counter-clockwise)
     */
    public double getRawSpinnerEncoderValue() {
        return -(spinner.getSelectedSensorPosition());
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
            * -1 //Negate the encoder
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

    public double getSpinnerPIDOutput() {
        return spinnerPIDOutput;
    }

    public double getSpinnerFeedForwardOutput() {
        return spinnerFeedForwardOutput;
    }

    /**
     * @return the output voltage on the motor
     */
    public double getSpinnerVoltage() {
        return spinner.getMotorOutputVoltage();
    }

    /**
     * @return the velocity the spinner is currently trying to achieve
     */
    public double getSpinnerTargetVelocity() {
        return spinnerPIDController.getSetpoint().velocity;
    }

    public double getSpinnnerVelocity() {
        return 
            spinner.getSelectedSensorVelocity() //Counts per 100 ms
            * 10 //Counts per second (TalonFX is weird)
            / COUNTS_SPINNER_ENCODER //Rotations per second
            * 360 //Degrees per second (at motor)
            * TURRET_GEAR_RATIO //Degrees per second (on turret)
            * -1 //Get the ratio correct for the direction the turret is moving
        ;
    }

    public Direction getSearchDirection() {return searchDirection;}
    public void setSearchDirection(Direction direction) {searchDirection = direction;}
    public void toggleSearchDirection() { 
        if(searchDirection == Direction.Clockwise) setSearchDirection(Direction.CounterClockwise);
        else setSearchDirection(Direction.Clockwise);
    }

    /* Other sensors */

    public LidarSensor getLidar() {
        return lidar;
    }

    public HallEffectSensor getCenterHallEffectSensor() {
        return centerHallEffect;
    }


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
        flywheelPIDOutput = flywheelPID.calculate(getFlyWheelRPM());
        flywheelFeedForwardOutput = flywheelFeedForward.calculate(flywheelPID.getSetpoint());

        if(getFlyWheelRPM()>=0) flywheel.setVoltage(flywheelPIDOutput + flywheelFeedForwardOutput);

        spinnerPIDOutput = spinnerPIDController.calculate(getSpinnerAngle(), spinnerSetpoint);
        spinnerFeedForwardOutput = spinnerFeedForward.calculate(spinnerPIDController.getSetpoint().velocity);

        spinner.setVoltage(spinnerPIDOutput + spinnerFeedForwardOutput);
    }

    public enum Direction {
        Clockwise, 
        CounterClockwise
    }


}
