package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberState;
import frc.robot.subsystems.Drivetrain.TeleopSpeeds;

public class MoveClimberCommand extends CommandBase{
    Climber climber;
    ClimberState state;
    Drivetrain drivetrain;
    
    public MoveClimberCommand(Climber climber, ClimberState state, Drivetrain d) {
        this.climber = climber;
        this.state = state;
        this.drivetrain = d;
    }

    @Override
    public void initialize() {
        climber.unBrake();
        climber.setClimberState(state);
        if (climber.isUp()){drivetrain.setSpeed(TeleopSpeeds.Slow);}
        else {drivetrain.setSpeed(TeleopSpeeds.Normal);}
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return !climber.isBusy() || climber.isForceStopped();
    }

    @Override
    public void end(boolean interrupted) {
        climber.brake();
    }
    
}
