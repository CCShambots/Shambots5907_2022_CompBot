package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;


public class LimeLightTurnCommand extends CommandBase{
    private Limelight limelight;
    private PIDController rotationPID;
    private Drivetrain driveTrain;

    private boolean finished = false;
    private double limelightOffset = 0;
    private double prevLimeLightOffset = 0;

    private double gyroSetpoint = 0;
    private boolean gyroSet = false;

    /**Command to turn automatically towards the target using the limelight */
    public LimeLightTurnCommand(Limelight limelight, Drivetrain driveTrain, PIDController xRPID) {
        rotationPID = xRPID;
        this.limelight = limelight;
        this.driveTrain = driveTrain;
 
        addRequirements(limelight);
        addRequirements(driveTrain);
    }

    /**Turns on the limelight, resets the gyro, and sets the limeLightOffset to whatever is currently on the network table */
    @Override
    public void initialize() {
        limelight.setOn();
        driveTrain.getGyro().setFusedHeading(0);

        limelightOffset = limelight.targetOffset().getX();
        prevLimeLightOffset = limelightOffset;
    }

    /** */
    @Override
    public void execute() {
        if (!gyroSet) limelightOffset = limelight.targetOffset().getX(); //Get the limelight offset from the network table as long as a value has not yet been passed to the gyro

        //Set the gyro setpoint if the limelight value has changed AND the limelight has detected a target  
        if(limelightOffset != prevLimeLightOffset && limelight.hasTarget()) {
            gyroSetpoint = limelightOffset;
            gyroSet = true;
        } else if(limelightOffset != prevLimeLightOffset) prevLimeLightOffset = limelightOffset; //If the limelight value has changed, but the limelight doesn't have a target, update prevLimelightOffset
        
        rotationPID.setSetpoint(gyroSetpoint);

        double move;

        /**If the limelight doesn't have a target, the robot needs to turn until a target is detected */
        if(limelight.hasTarget()){
            move = rotationPID.calculate(driveTrain.getGyroHeading());
        } else {
            move = Constants.LIMELIGHT_FIND_TARGET_SPEED;
        }

        driveTrain.arcadeDrive(0, move);

        //Indicate that the command is finished if the robot is within 5 degrees and the gyro has received an updated set point from the limelight
        if(Math.abs(driveTrain.getGyroHeading() - gyroSetpoint) < Constants.LIMELIGHT_TOLERANCE && gyroSet) finished = true; 
    }


    @Override
    public boolean isFinished() {
        return finished;
    }

    /**Turn off the limelight */
    @Override
    public void end(boolean interrupted) {
        limelight.setOff();
    }
    
}
