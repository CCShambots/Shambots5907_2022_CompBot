package frc.robot.commands.auton;

import java.util.Map;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.TrajectoryCommand;
import frc.robot.commands.turret.MoveSpinnerCommand;
import frc.robot.commands.turret.ShootCommand;
import frc.robot.commands.turret.SpinUpFlywheelCommand;
import frc.robot.commands.turret.limelight.AutonomousTargetCommand;
import frc.robot.util.auton.AllRobotSubsystems;
import static frc.robot.util.auton.AutoRoutes.Trajectories;

import static frc.robot.Constants.Turret.*;


public class BackUpLeftRoute extends BaseRoute{

    public BackUpLeftRoute(AllRobotSubsystems subsystems, Map<Trajectories, Trajectory> paths) {
        super(subsystems, paths);

        addCommands(
            setupAuto(paths.get(Trajectories.BackUpLeftRoute)),
            new WaitCommand(8),
            new ParallelCommandGroup(
                new MoveSpinnerCommand(turret, 0),
                new SpinUpFlywheelCommand(turret, FLYWHEEL_HIGH_RPM),
          
                new TrajectoryCommand(drivetrain, paths.get(Trajectories.BackUpLeftRoute))
            ),
            new AutonomousTargetCommand(turret),
            new ShootCommand(conveyor),
            new InstantCommand(() -> {
                turret.setFlywheelTarget(0);
                turret.setSpinnerTarget(0);
            })
        );
    }
    
}
