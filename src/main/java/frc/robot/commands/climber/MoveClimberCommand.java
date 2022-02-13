package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberState;

public class MoveClimberCommand extends CommandBase{
    Climber climber;
    ClimberState state;
    
    public MoveClimberCommand(Climber climber, ClimberState state) {
        this.climber = climber;
        this.state = state;
    }

    @Override
    public void initialize() {
        climber.unBrake();
        climber.setClimberState(state);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return !climber.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        climber.brake();
    }
    
}
