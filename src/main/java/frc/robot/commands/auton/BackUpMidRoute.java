package frc.robot.commands.auton;

import java.util.Map;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.TrajectoryCommand;
import frc.robot.commands.turret.MoveSpinnerCommand;
import frc.robot.commands.turret.SpinUpFlywheelCommand;
import frc.robot.commands.turret.limelight.AutonomousTargetCommand;
import frc.robot.commands.turret.shooting.ShootCommand;
import frc.robot.util.auton.AllRobotSubsystems;
import static frc.robot.util.auton.AutoRoutes.Trajectories;

import static frc.robot.Constants.Turret.*;


public class BackUpMidRoute extends BaseRoute{

    public BackUpMidRoute(AllRobotSubsystems subsystems, Map<Trajectories, Trajectory> paths) {
        super(subsystems, paths);

        addCommands(
            setupAuto(paths.get(Trajectories.BackUpMidRoute)),
            new WaitCommand(8),
            new ParallelCommandGroup(
                new MoveSpinnerCommand(turret, 0),
                new SpinUpFlywheelCommand(turret, FLYWHEEL_HIGH_RPM),
                new PrintCommand("Going onwards to the next thing"),
                new TrajectoryCommand(drivetrain, paths.get(Trajectories.BackUpMidRoute))
            ),
            new AutonomousTargetCommand(turret),
            new ShootCommand(conveyor, turret),
            new InstantCommand(() -> {
                turret.setFlywheelTarget(0);
                turret.setSpinnerTarget(0);
            })
        );
    }
    
}
