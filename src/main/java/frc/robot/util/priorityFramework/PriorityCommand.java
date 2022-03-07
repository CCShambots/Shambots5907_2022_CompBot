package frc.robot.util.priorityFramework;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PriorityCommand extends CommandBase{
    private Command command;
    private Set<PrioritizedSubsystem> requirements;
    private int priority;
    private BooleanSupplier condition;

    public PriorityCommand(Command command, int priority, BooleanSupplier condition) {
        this.command = command;
        //Get the requirements of the command and add them to the Set of requirements
        requirements = new HashSet<>();
        requirements.addAll(getPrioritizedSubsystems(command.getRequirements()));
        this.priority = priority;
        this.condition = condition;
    }

    public PriorityCommand(Command command, int priority) {
        this(command, priority, () -> false);
    }

    public PriorityCommand(Command command, int priority, BooleanSupplier condition, PrioritizedSubsystem... requirements) {
        this(command, priority, condition);
        Collections.addAll(this.requirements, requirements);
    }

    public PriorityCommand(Command command, int priority, PrioritizedSubsystem... requirements) {
        this(command, priority, () -> false, requirements);
    }

    /**
     * Attempts
     */
    @Override
    public void initialize() {
        //Attempt to shchedule the command

        if(!condition.getAsBoolean()) return;

        for(PrioritizedSubsystem s : requirements) {
            if(! (this.priority > s.getPriority())) return;
        }

        //Add a decorator to the command that wil reset the subsystems to -1 priority after the command finishes
        command.andThen(new ResetSubsystemsCommand(requirements));
        command.schedule();
    }

    @Override
    public boolean isFinished() {return true;}


    private PrioritizedSubsystem prioritizedSubsystemFromSubsystem(Subsystem s) throws Exception{
        return (PrioritizedSubsystem) s;
    }

    private Set<PrioritizedSubsystem> getPrioritizedSubsystems(Set<Subsystem> subsystems) {
        Set<PrioritizedSubsystem> prioritizedSubsystems = new HashSet<>();

        for(Subsystem s : subsystems) {
            try {
                PrioritizedSubsystem p = prioritizedSubsystemFromSubsystem(s);

                prioritizedSubsystems.add(p);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        return prioritizedSubsystems;
    }

    
}
