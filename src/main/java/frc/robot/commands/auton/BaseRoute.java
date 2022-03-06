package frc.robot.commands.auton;

import java.util.Map;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.util.auton.AllRobotSubsystems;
import frc.robot.util.auton.AutoRoutes.Trajectories;

public class BaseRoute extends SequentialCommandGroup{
    protected Drivetrain drivetrain;
    protected Intake intake;
    protected Conveyor conveyor;
    protected Turret turret;
    protected Climber climber;

    protected Map<Trajectories, Trajectory> paths;

    //TODO: Simple, delayed auto that shoots one ball

    public BaseRoute(AllRobotSubsystems subsystems, Map<Trajectories, Trajectory> paths) {
        drivetrain = subsystems.getDrivetrain();
        intake = subsystems.getIntake();
        conveyor = subsystems.getConveyor();
        turret  = subsystems.getTurret();
        climber = subsystems.getClimber();
        this.paths = paths;


        addRequirements(drivetrain, intake, conveyor, turret);
        //TODO: Add climber back once we add it
        // addRequirements(drivetrain, intake, conveyor, turret, climber);
    }

    protected SequentialCommandGroup setupAuto(Trajectory trajectory) {
        return new SequentialCommandGroup(
          new InstantCommand(() -> {
            drivetrain.resetOdometry(trajectory.getInitialPose());
            conveyor.setupTrackerAuto();
            turret.resetSpinnerAngle(0);
            turret.setKnowsLocation(true);
          })
        );
    }
}
