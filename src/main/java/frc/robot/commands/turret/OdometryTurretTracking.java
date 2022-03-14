package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.turret.limelight.TeleopTrackingCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Turret.Direction;

import static frc.robot.Constants.*;
import static java.lang.Math.*;

/**
 * Command for tracking the goal roughly based on the robot's pose (as recorded by odometry)
 */
public class OdometryTurretTracking extends CommandBase{
    private Drivetrain drivetrain;
    private Conveyor conveyor;
    private Turret turret;

    private double distanceToGoalToTrigger = 5;
    private Intake intake;

    public OdometryTurretTracking(Drivetrain drivetrain, Intake intake, Conveyor conveyor, Turret turret) {
        this.drivetrain = drivetrain;
        this.conveyor = conveyor;
        this.turret = turret;
        this.intake = intake;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(conveyor.getNumberOfBalls() > 0) {

            //Tracking with odometry
            Pose2d poseFeet = poseMetersToFeet(drivetrain.getOdometryPose());
            double targetAngle = getTurretAngleFromPose(poseFeet);
            turret.setSpinnerTarget(targetAngle);

            //Switch to tracking with
            if(poseFeet.getTranslation().getDistance(goalPos) <= distanceToGoalToTrigger) {
                //Begin teleop tracking command
                //TODO: Make priority command
                new ConditionalCommand(new TeleopTrackingCommand(turret, conveyor, intake), new InstantCommand(), () -> turret.knowsLocation()).schedule();
            } 
        }
    }

    private double getTurretAngleFromPose(Pose2d robotPose) {
        double xOffset = goalPos.getX() - robotPose.getX();
        double yOffset = goalPos.getY() - robotPose.getY();

        //Angle to the goal (regardless of the robot's direction)
        double angleToGoal = atan2(yOffset, xOffset);

        double robotAngleRadians = robotPose.getRotation().getRadians();
        double relAngle = -(robotAngleRadians - angleToGoal);

        if(relAngle > PI) relAngle = relAngle - toRadians(360);
        else if(relAngle < -PI) relAngle = relAngle + toRadians(360);

        if(relAngle > turret.getSpinnerAngle()) turret.setSearchDirection(Direction.Clockwise);
        else turret.setSearchDirection(Direction.CounterClockwise);

        return toDegrees(relAngle);
    }

    private Pose2d poseMetersToFeet(Pose2d initialPose) {
        return new Pose2d(Units.metersToFeet(initialPose.getX()), Units.metersToFeet(initialPose.getY()), initialPose.getRotation());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    //There's nothing to implement in end
}
