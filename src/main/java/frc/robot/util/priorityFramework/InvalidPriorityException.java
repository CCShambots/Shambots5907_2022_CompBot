package frc.robot.util.priorityFramework;

public class InvalidPriorityException extends Exception{

    public InvalidPriorityException() {
        super("Commands cannot have a priority below 1");
    }
    
}
