package frc.robot.commands.auton;

import java.util.Map;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.commands.drivetrain.TrajectoryCommand;
import frc.robot.util.auton.AllRobotSubsystems;
import frc.robot.util.auton.AutoRoutes.Trajectories;

public class MeterRoute extends BaseRoute{

    public MeterRoute(AllRobotSubsystems subsystems, Map<Trajectories, Trajectory> paths) {
        super(subsystems, paths);
        
        
        addCommands(
            setupAuto(paths.get(Trajectories.Meter)),
            new TrajectoryCommand(drivetrain, paths.get(Trajectories.Meter))
        );
    }

    
}
