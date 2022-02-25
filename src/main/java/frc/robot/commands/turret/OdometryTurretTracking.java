package frc.robot.commands.turret;

import javax.sql.rowset.spi.TransactionalWriter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

import static frc.robot.Constants.*;

/**
 * Command for tracking the goal roughly based on the robot's pose (as recorded by odometry)
 */
public class OdometryTurretTracking extends CommandBase{
    private Drivetrain drivetrain;
    private Conveyor conveyor;
    private Turret turret;

    public OdometryTurretTracking(Drivetrain drivetrain, Conveyor conveyor, Turret turret) {
        this.drivetrain = drivetrain;
        this.conveyor = conveyor;
        this.turret = turret;

        addRequirements(turret);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if(conveyor.getNumberOfBalls() == 2) {
            turret.setSpinnerTarget(getTurretAngleFromPose(drivetrain.getOdometryPose()));
        }
    }

    private double getTurretAngleFromPose(Pose2d robotPose) {
        return 0;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    //There's nothing to implement in end
}
