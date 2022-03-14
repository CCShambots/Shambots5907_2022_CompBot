package frc.robot.util.priorityFramework;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Not to be used outside of the framework, otherwise errors in scheduling could occur
 */
class ResetSubsystemsCommand extends CommandBase{

    private Set<PrioritizedSubsystem> subsystems;
    private int priority;

    public ResetSubsystemsCommand(Set<PrioritizedSubsystem> subsystems, int priority) {
        this.subsystems = subsystems;
        this.priority = priority;
    }

    @Override
    public void initialize()
    {
        for(PrioritizedSubsystem s : subsystems) {
            //Only reset the subsystem if it has not yet been set to a new, higher priority
            if(! (s.getPriority() > priority)) s.resetPriority();
        }
    }    

    @Override
    public boolean isFinished() {
        return true;
    }
}
