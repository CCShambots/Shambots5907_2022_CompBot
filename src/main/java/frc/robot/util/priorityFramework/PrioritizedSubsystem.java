package frc.robot.util.priorityFramework;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Base class for a prioritized subsystem that can run PriorityCommands. 
 */
public class PrioritizedSubsystem extends SubsystemBase{
    
    private int priority;

    public PrioritizedSubsystem() {
        super();

        priority = -1;
    }

    /**
     * Sets the priority of the command currently running on the subsystem. Should only be accessed from the PriorityCommand class
     * @param priority the priority of the command being scheduled
     * @throws InvalidPriorityException If the priority is less than 1, the method will throw an error
     */
    void setPriority(int priority) throws InvalidPriorityException {
        if(priority < 1) throw new InvalidPriorityException();
        this.priority = priority;
    }

    /**
     * Resets the subsystem to have a priority of -1 (a command is no longer running)
     */
    void resetPriority() {
        this.priority = -1;
    }


    /**
     * Gets the priority of the current command running on the subsystem.
     * @return the priority of the command currently running on the subsystem (-1 if no command is running)
     */
    public int getPriority() {
        return priority;
    }

    /**
     * 
     * @param cls The command to check
     * @return true if the subsystem is running an instance of the command passed as parameter
     * @throws NotACommandException
     */
    public boolean isRunning(Class<?> cls) throws NotACommandException{
        if(!Command.class.isAssignableFrom(cls)) throw new NotACommandException("isRunning()");

        if(getCurrentCommand() == null) return false;
        return cls.isInstance(getCurrentCommand());
    }

    /**
     * 
     * @param command
     * @return true if the subsystem is currently running that instance of the command
     */
    public boolean isRunning(Command command) {
        if(getCurrentCommand() == null) return false; 
        return getCurrentCommand().equals(command);
    }
}
