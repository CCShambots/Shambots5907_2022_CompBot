package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.DriveModes;

public class DrivingCommand extends CommandBase{
    private Drivetrain drivetrain;
    private Supplier<Double> leftX;
    private Supplier<Double> leftY;
    private Supplier<Double> rightY;

    public DrivingCommand(Drivetrain drivetrain, Supplier<Double> leftX, Supplier<Double> leftY, Supplier<Double> rightY) {
        this.drivetrain = drivetrain;
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightY = rightY;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if(drivetrain.getDriveMode() == DriveModes.Tank) {
            drivetrain.tankDriveJoystick(leftY.get(), rightY.get());
        } else if(drivetrain.getDriveMode() == DriveModes.Arcade) {
            drivetrain.arcadeDriveJoysticks(leftY.get(), leftX.get());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.tankDriveJoystick(0, 0);
    }
    
}
