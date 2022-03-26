package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class RetractClimberSolenoidsCommand extends CommandBase{
    
    private Climber climber;

    public RetractClimberSolenoidsCommand(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void initialize() {
        climber.setSolenoids(Value.kReverse);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
