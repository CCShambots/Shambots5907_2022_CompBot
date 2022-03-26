package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ExtendClimberSolenoidsCommand extends CommandBase{

    private Climber climber;

    public ExtendClimberSolenoidsCommand(Climber climber) {
        this.climber = climber;

    }

    @Override
    public void initialize() {
        climber.setSolenoids(Value.kForward);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
