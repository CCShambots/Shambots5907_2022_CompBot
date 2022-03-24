package frc.robot.commands.auton;

import java.util.Map;


import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.TrajectoryCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.turret.ShootCommand;
import frc.robot.commands.turret.SpinUpFlywheelCommand;
import frc.robot.commands.turret.limelight.AutonomousTargetCommand;
import frc.robot.subsystems.Turret.Direction;
import frc.robot.util.auton.AllRobotSubsystems;
import frc.robot.util.auton.AutoRoutes.Trajectories;

import static frc.robot.util.auton.AutoRoutes.Trajectories.*;
import static frc.robot.Constants.Turret.*;

public class FourBallRoute extends BaseRoute{

    public FourBallRoute(AllRobotSubsystems subsystems, Map<Trajectories, Trajectory> paths) {
        super(subsystems, paths);
        
        
        addCommands(
            setupAuto(paths.get(FourBall1)),
            new ParallelCommandGroup(
                new InstantCommand(() -> {
                    turret.setSpinnerTarget(-190);
                    turret.setSearchDirection(Direction.Clockwise);
                }),
                new SpinUpFlywheelCommand(turret, FLYWHEEL_HIGH_RPM + 350),
                new IntakeCommand(intake, conveyor),
                new SequentialCommandGroup(
                    new TrajectoryCommand(drivetrain, paths.get(FourBall1)),
                    new WaitCommand(1),
                    new InstantCommand(() -> intake.setShouldEnd(true))
                
                )
            ),
            new AutonomousTargetCommand(turret),
            new ShootCommand(conveyor),
            new ParallelCommandGroup(
                new InstantCommand(() -> turret.setSpinnerTarget(-180)),
                new IntakeCommand(intake, conveyor),
                new SequentialCommandGroup(
                    new TrajectoryCommand(drivetrain, paths.get(FourBall2)),
                    new WaitCommand(2.0),
                    new InstantCommand(() -> intake.setShouldEnd(true))
                )
            ),
            new TrajectoryCommand(drivetrain, paths.get(FourBall3)),
            new AutonomousTargetCommand(turret),
            new ShootCommand(conveyor),
            new InstantCommand(() -> {
                turret.setFlywheelTarget(0);
                turret.setSpinnerTarget(0);
            })
        );
    }


    
}
