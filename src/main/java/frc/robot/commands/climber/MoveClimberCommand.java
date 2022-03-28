package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberState;
import frc.robot.subsystems.Climber.ControlLoopType;
import frc.robot.subsystems.Drivetrain.TeleopSpeeds;

public class MoveClimberCommand extends CommandBase{
    private Climber climber;
    private ClimberState state;
    private Drivetrain drivetrain;
    private ControlLoopType type;
    private boolean brakeAtEnd;
    
    public MoveClimberCommand(Climber climber, Drivetrain drivetrain, ClimberState state, ControlLoopType type, boolean brakeAtEnd) {
        this.climber = climber;
        this.state = state;
        this.drivetrain = drivetrain;
        this.type = type;
        this.brakeAtEnd = brakeAtEnd;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.unBrake();
        climber.setClimberState(state, type, true);
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
        if(brakeAtEnd) climber.brake();
    }
}
