package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
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
    public void initialize() {}

    @Override
    public void execute() {
        if(drivetrain.getDriveMode() == DriveModes.Tank) tankDrive();
        else if(drivetrain.getDriveMode() == DriveModes.Arcade) arcadeDrive();
    }

    private void tankDrive() {
        drivetrain.tankDrive(smoothing1.calculate(adjustJoystick(leftY.get())), smoothing2.calculate(adjustJoystick(rightY.get())));;
    }

    private void arcadeDrive() {
        drivetrain.arcadeDrive(smoothing1.calculate(adjustJoystick(leftY.get())), smoothing2.calculate(adjustJoystick(leftX.get())));
    }

    /**
   * 
   * @param input Raw joystick input
   * @return the input after being sped up, slowed down, or reversed, as per the current drivetrain settings
   */
  private double adjustJoystick(double input) {
        input *= -1;

        //Create dead zones
        if(Math.abs(input) < 0.05) return 0;

        double output = input * Math.abs(input);

        output *= drivetrain.getMaxSpeed();

        output *= drivetrain.isReversed() ? -1 : 1;

        if(drivetrain.getSpeedMode() == TeleopSpeeds.Normal) output *= NORMAL_SPEED_MULT;

        return output;
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
