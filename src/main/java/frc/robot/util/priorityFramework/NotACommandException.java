package frc.robot.util.priorityFramework;

public class NotACommandException extends Exception{

    public NotACommandException(String methodName) {
        super("The class you passed to method " + methodName + "is not a command!");
    }
    
}
