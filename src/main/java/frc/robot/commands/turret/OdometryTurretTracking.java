package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
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
        if(conveyor.getNumberOfBalls() > 0 && drivetrain.shouldUseOdometry()) {

            //Tracking with odometry
            Pose2d poseFeet = drivetrain.getOdometryPoseFeet();
            double targetAngle = getTurretAngleFromPose(poseFeet);
            turret.setSpinnerTarget(targetAngle);

            //If the turret was told that it should end, turn that false off if the robot leaves the allowed starting area
            // if(turret.getShouldEndTargeting()) turret.setShouldEndTargeting(!(distanceToGoal > AUTOMATIC_START_DISTANCE));
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

    @Override
    public boolean isFinished() {
        return false;
    }

    //There's nothing to implement in end
}
