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

public class CSGO1Route extends BaseRoute{

    public CSGO1Route(AllRobotSubsystems subsystems, Map<Trajectories, Trajectory> paths) {
        super(subsystems, paths);
        
        System.out.println("Paths size: " + paths.size());

        for(Trajectories t : paths.keySet()) {
            System.out.println("Found path: " + t.name());
        }

        addCommands(
            setupAuto(paths.get(CSGO1)),
            new ParallelCommandGroup(
                // new SequentialCommandGroup(
                //     // new ZeroSpinnerCommand(turret, 45),
                //     // new MoveSpinnerCommand(turret, 0)
                // ),
                new SpinUpFlywheelCommand(turret, 2000), //TODO: Get an actual target for this
          
                new IntakeCommand(intake, conveyor),

                new SequentialCommandGroup(                    
                    new TrajectoryCommand(drivetrain, paths.get(CSGO1)),
                    new InstantCommand(() -> intake.setShouldEnd(true))
                )
            ),
            new AutonomousTargetCommand(turret),
            new ShootCommand(conveyor, Amount.Two)
        );
    }

    
}
