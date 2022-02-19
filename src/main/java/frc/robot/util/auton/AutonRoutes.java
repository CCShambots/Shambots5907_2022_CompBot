package frc.robot.util.auton;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auton.CSGO1Route;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

public class AutonRoutes {

    Map<Trajectories, Trajectory> paths;
    Map<AutoPaths, Command> autoRoutes;
    AllRobotSubsystems allRobotSubsystems;

    public AutonRoutes(Map<String, Trajectory> trajectories, Drivetrain drivetrain, Intake intake, Conveyor conveyor, Turret turret, Climber climber) {
        allRobotSubsystems = new AllRobotSubsystems(drivetrain, intake, conveyor, turret, climber);

        paths = new HashMap<>();

        for(String s : trajectories.keySet()) {
            paths.put(Trajectories.valueOf(s), trajectories.get(s)); 
        }

        autoRoutes = new HashMap<>();
        autoRoutes.put(AutoPaths.CSGO1, new CSGO1Route(allRobotSubsystems, paths));
    }

    public Map<Trajectories, Trajectory> getTrajectories() {
        return paths;
    } 

    public Map<AutoPaths, Command> getAutoRoutes() {
        return autoRoutes;
    }


    public static enum AutoPaths {
        CSGO1,
        CSGO2,
        CSGO3
    }

    public static enum Trajectories {
        CSGO1,
        CSGO2,
        CSGO31,
        CSGO32
    }

}
