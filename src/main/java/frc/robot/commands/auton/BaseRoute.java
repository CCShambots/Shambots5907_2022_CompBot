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

    public BaseRoute(AllRobotSubsystems subsystems, Map<Trajectories, Trajectory> paths) {
        drivetrain = subsystems.getDrivetrain();
        intake = subsystems.getIntake();
        conveyor = subsystems.getConveyor();
        turret  = subsystems.getTurret();
        climber = subsystems.getClimber();
        this.paths = paths;


        addRequirements(drivetrain, intake, conveyor, turret);
    }

    protected SequentialCommandGroup setupAuto(Trajectory trajectory) {
        return new SequentialCommandGroup(
          new InstantCommand(() -> {
            drivetrain.resetOdometry(trajectory.getInitialPose());
            drivetrain.setDefending(true);
            drivetrain.setUseOdometry(true);
            conveyor.setupTrackerAuto();
            turret.resetSpinnerAngle(0);
            turret.setKnowsLocation(true);
          })
        );
    }

    protected InstantCommand endAuto() {
      return new InstantCommand(() -> {
        turret.setSpinnerTarget(0);
        turret.setFlywheelTarget(0);
        drivetrain.setDefending(false);
      });
    }
}
