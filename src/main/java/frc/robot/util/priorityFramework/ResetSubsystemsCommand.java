package frc.robot.util.priorityFramework;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Not to be used outside of the framework, otherwise errors in scheduling could occur
 */
public class ResetSubsystemsCommand extends CommandBase{

    Set<PrioritizedSubsystem> subsystems;

    public ResetSubsystemsCommand(Set<PrioritizedSubsystem> subsystems) {
        this.subsystems = subsystems;
    }

    @Override
    public void initialize()
    {
        for(PrioritizedSubsystem s : subsystems) {
            s.resetPriority();
        }
    }    

    @Override
    public boolean isFinished() {
        return true;
    }
}
