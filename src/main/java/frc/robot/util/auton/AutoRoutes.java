package frc.robot.util.auton;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auton.CSGO1Route;
import frc.robot.commands.auton.CSGO2Route;
import frc.robot.commands.auton.CSGO3Route;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

public class AutoRoutes {

    Map<Trajectories, Trajectory> paths;
    Map<Object, Command> autoRoutes;
    AllRobotSubsystems allRobotSubsystems;

    public AutoRoutes(Map<String, Trajectory> trajectories, Drivetrain drivetrain, Intake intake, Conveyor conveyor, Turret turret) {
        allRobotSubsystems = new AllRobotSubsystems(drivetrain, intake, conveyor, turret);

        paths = new HashMap<>();

        for(String s : trajectories.keySet()) {
            paths.put(Trajectories.valueOf(s), trajectories.get(s)); 
        }


        autoRoutes = new HashMap<>();
        autoRoutes.put(AutoPaths.CSGO1, new CSGO1Route(allRobotSubsystems, paths));
        autoRoutes.put(AutoPaths.CSGO2, new CSGO2Route(allRobotSubsystems, paths));
        autoRoutes.put(AutoPaths.CSGO3, new CSGO3Route(allRobotSubsystems, paths));
    }

    public Map<Trajectories, Trajectory> getTrajectories() {
        return paths;
    } 

    public Map<Object, Command> getAutoRoutes() {
        return autoRoutes;
    }

    public Trajectory getFirstTrajectory(AutoPaths selectedPath) {
        switch (selectedPath) {
            case CSGO1: return paths.get(Trajectories.CSGO1);
            case CSGO2: return paths.get(Trajectories.CSGO2);
            case CSGO3: return paths.get(Trajectories.CSGO31);
            case BackUp1: return paths.get(Trajectories.BackUp1Route);
            case BackUp2: return paths.get(Trajectories.BackUp2Route);
        }

        return null;
    }


    public static enum AutoPaths {
        CSGO1,
        CSGO2,
        CSGO3,
        BackUp1,
        BackUp2
    }

    public static enum Trajectories {
        CSGO1,
        CSGO2,
        CSGO31,
        CSGO32,
        BackUp1Route,
        BackUp2Route
    }

}
