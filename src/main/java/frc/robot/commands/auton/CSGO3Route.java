package frc.robot.commands.auton;

import static frc.robot.util.auton.AutoRoutes.Trajectories.*;

import java.util.Map;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.TrajectoryCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.limelight.AutonomousTargetCommand;
import frc.robot.commands.turret.MoveSpinnerCommand;
import frc.robot.commands.turret.ShootCommand;
import frc.robot.commands.turret.SpinUpFlywheelCommand;
import frc.robot.commands.turret.ZeroSpinnerCommand;
import frc.robot.commands.turret.ShootCommand.Amount;
import frc.robot.util.auton.AllRobotSubsystems;
import frc.robot.util.auton.AutoRoutes.Trajectories;

import static frc.robot.Constants.Turret.*;


public class CSGO3Route extends BaseRoute{

    public CSGO3Route(AllRobotSubsystems subsystems, Map<Trajectories, Trajectory> paths) {
        super(subsystems, paths);


        addCommands(
            setupAuto(paths.get(CSGO31)),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    // new ZeroSpinnerCommand(turret, 45),
                    new MoveSpinnerCommand(turret, 0)
                ),
                new SpinUpFlywheelCommand(turret, FLYWHEEL_TARGET_RPM),
          
                new IntakeCommand(intake, conveyor),

                new SequentialCommandGroup(                    
                    new TrajectoryCommand(drivetrain, paths.get(CSGO31)),
                    new InstantCommand(() -> intake.setShouldEnd(true)),
                    new TrajectoryCommand(drivetrain, paths.get(CSGO32))
                )
            ),
            new AutonomousTargetCommand(turret),
            new ShootCommand(conveyor)
        );
    }

    
}
