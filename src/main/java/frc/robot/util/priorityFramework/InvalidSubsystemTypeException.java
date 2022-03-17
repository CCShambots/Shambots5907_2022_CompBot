package frc.robot.util.priorityFramework;

public class InvalidSubsystemTypeException extends Exception{

    public InvalidSubsystemTypeException(String name) {
        super("Subsystem " + name + " does not extend PrioritizedSubsystem!");
    }
    
}
