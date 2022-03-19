package frc.robot.util.priorityFramework;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;

public class PriorityRegistry {

    public static Map<Class<?>, Integer> registry = new HashMap<>();

    public static void registerCommand(Class<?> cls, Integer priority) throws NotACommandException {
        if(!Command.class.isAssignableFrom(cls)) throw new NotACommandException("registerCommand()");

        registry.put(cls, priority);
    }

    public static Integer lookUpCommand(Class<?> cls) {return registry.get(cls);}
    public static Integer lookUpCommand(Command command) {return registry.get(command.getClass());}
    
}
