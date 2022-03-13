package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.DriveModes;
import frc.robot.subsystems.Drivetrain.TeleopSpeeds;
import static frc.robot.Constants.Drivetrain.*;

public class DrivingCommand extends CommandBase{
    private Drivetrain drivetrain;
    private Supplier<Double> leftX;
    private Supplier<Double> leftY;
    private Supplier<Double> rightY;

    SlewRateLimiter smoothing1;
    SlewRateLimiter smoothing2;

    public DrivingCommand(Drivetrain drivetrain, Supplier<Double> leftX, Supplier<Double> leftY, Supplier<Double> rightY) {
        this.drivetrain = drivetrain;
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightY = rightY;

        addRequirements(drivetrain);

        smoothing1 = new SlewRateLimiter(drivetrain.getSmoothing());
        smoothing2 = new SlewRateLimiter(drivetrain.getSmoothing());
    }

    @Override
    public void execute() {
        if(drivetrain.getDriveMode() == DriveModes.Tank) tankDrive();
        else if(drivetrain.getDriveMode() == DriveModes.Curvature) curvatureDrive();
    }

    /**
     * Method that runs the full loop for tank driving
     */
    private void tankDrive() {
        drivetrain.tankDrive(
            -smoothing1.calculate(joystickToMeters(adjustJoystick(leftY.get()))), 
            -smoothing2.calculate(joystickToMeters(adjustJoystick(rightY.get()))));
    }


    /**
     * Method that runs the arcade drive loop
     */
    public void curvatureDrive() {
        //True is passed to allow turning in place
        WheelSpeeds speeds = DifferentialDrive.curvatureDriveIK(
            -smoothing1.calculate(adjustJoystick(leftY.get())), 
            smoothing2.calculate(adjustJoystick(leftX.get())), true);
    
        //The speeds are within -1 and 1, so they must be multiplied out for PID velocity control
        drivetrain.tankDrive(joystickToMeters(speeds.left), joystickToMeters(speeds.right));
    }

    /**
     * Adjusts the joystick by deadzoning, reversing, etc
     * @param input Raw joystick input
     * @return the adjusted joystick (in range -1.0 to 1.0)
     */
    private double adjustJoystick(double input) {
        //Create dead zones
        if(Math.abs(input) < 0.05) return 0;

        double output = input * Math.abs(input);

        output *= drivetrain.isReversed() ? -1 : 1;

        if(drivetrain.getSpeedMode() == TeleopSpeeds.Normal) output *= NORMAL_SPEED_MULT;
        else if(drivetrain.getSpeedMode() == TeleopSpeeds.Slow) output *= SLOW_SPEED_MULT;

        return output;
    }

    /**
     * 
     * @param input joystick value between -1.0 and 1.0
     * @return the vloeicyt that corresponds to that based on maximum drivetrain velocity
     */
    private double joystickToMeters(double input) {
        return input * drivetrain.getMaxSpeed();
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.tankDrive(0, 0);
    }
    
}
