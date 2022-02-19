package frc.robot.util;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

public class AutonRoutes {

    Map<Trajectories, Trajectory> trajectories;
    AllRobotSubsystems allRobotSubsystems;

    public AutonRoutes(Map<String, Trajectory> trajectories, Drivetrain drivetrain, Intake intake, Conveyor conveyor, Turret turret, Climber climber) {
        allRobotSubsystems = new AllRobotSubsystems(drivetrain, intake, conveyor, turret, climber);

        this.trajectories = new HashMap<>();

        for(String s : trajectories.keySet()) {
            this.trajectories.put(Trajectories.valueOf(s), trajectories.get(s)); 
        }
    }

    public Map<Trajectories, Trajectory> getRoutes() {
        return trajectories;
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
