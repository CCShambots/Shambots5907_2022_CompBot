package frc.robot.util;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousCommand extends SequentialCommandGroup{

    private String id;

    public AutonomousCommand(String identifier, Command... commands) {
        super(commands);

        id = identifier;
    }

    public static AutonomousCommand getCommandById(List<AutonomousCommand> commands, String id) {
        for(AutonomousCommand command : commands) {
            if(command.id.equals(id)) return command;
        }

        return null;
    }
    
}
