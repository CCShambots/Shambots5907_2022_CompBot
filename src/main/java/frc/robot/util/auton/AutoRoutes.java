package frc.robot.util.auton;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auton.BackUpLeftRoute;
import frc.robot.commands.auton.BackUpMidRoute;
import frc.robot.commands.auton.BackUpRightRoute;
import frc.robot.commands.auton.CSGO1Route;
import frc.robot.commands.auton.CSGO2Route;
import frc.robot.commands.auton.CSGO3Route;
import frc.robot.commands.auton.MeterRoute;
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
        autoRoutes.put(AutoPaths.BackUpLeft, new BackUpLeftRoute(allRobotSubsystems, paths));
        autoRoutes.put(AutoPaths.BackUpMid, new BackUpMidRoute(allRobotSubsystems, paths));
        autoRoutes.put(AutoPaths.BackUpRight, new BackUpRightRoute(allRobotSubsystems, paths));
        autoRoutes.put(AutoPaths.Meter, new MeterRoute(allRobotSubsystems, paths));

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
            case BackUpLeft: return paths.get(Trajectories.BackUpLeftRoute);
            case BackUpMid: return paths.get(Trajectories.BackUpMidRoute);
            case BackUpRight: return paths.get(Trajectories.BackUpRightRoute);
            case Meter: return paths.get(Trajectories.Meter);
        }

        return null;
    }


    public static enum AutoPaths {
        CSGO1,
        CSGO2,
        CSGO3,
        BackUpLeft,
        BackUpMid,
        BackUpRight,
        Meter,
        FourBall
    }

    public static enum Trajectories {
        CSGO1,
        CSGO2,
        CSGO31,
        CSGO32,
        BackUpLeftRoute,
        BackUpMidRoute,
        BackUpRightRoute,
        Meter,
        FourBall1,
        FourBall2,
        FourBall3
    }

}
