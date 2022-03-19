package frc.robot.commands.drivetrain;

import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.Drivetrain.*;

public class TrajectoryCommand extends CommandBase{
    private Drivetrain drivetrain;

    //Objects for the Ramsete Controller
    private Trajectory trajectory;
    private RamseteController controller;
    private DifferentialDriveKinematics kDriveKinematics;

    //Variables for controlling the command
    private States state = States.Following;
    private Timer timer;

    private double timeToReachEndpoint = .5; //Seconds allowed for the robot to reach the end of the trajectory

    private boolean finished = false;

    public TrajectoryCommand(Drivetrain drivetrain, Pose2d startPose, List<Translation2d> passThroughPoints, Pose2d endPose) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        kDriveKinematics = new DifferentialDriveKinematics(TRACK_WIDTH);

        // Create a voltage constraint to ensure we don't accelerate too fast
        DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            LEFT_KS,
            LEFT_KV),
        kDriveKinematics,
        MAX_VOLTAGE);

        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                    MAX_LINEAR_VELOCITY,
                    MAX_LINEAR_ACCELERATION)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(kDriveKinematics);
        
        trajectory = TrajectoryGenerator.generateTrajectory(startPose, passThroughPoints, endPose, config);

        controller = new RamseteController(K_RAMSETE_B, K_RAMSETE_ZETA);

    }

    /**
     * Alternate constructor for trajectories grabbed from pathweaver.
     * @param drivetrain
     * @param trajectory
     */
    public TrajectoryCommand(Drivetrain drivetrain, Trajectory trajectory) {
        addRequirements(drivetrain);

        this.trajectory = trajectory;
        this.drivetrain = drivetrain;
        kDriveKinematics = new DifferentialDriveKinematics(TRACK_WIDTH);
        controller = new RamseteController(K_RAMSETE_B, K_RAMSETE_ZETA);
    }
    @Override
    public void initialize() {
        finished = false;
        
        timer = new Timer();
        timer.start();
    }

    @Override
    public void execute() {
        
        double runTime = timer.get();

        //This loop is run while the robot is still following the trajectory
        if(state == States.Following) {
            //The time spent following has reached the time the trajectory should take
            if(runTime >= trajectory.getTotalTimeSeconds()) {
                state = States.ReachEndpoint;
                return;
            }

            sendMotorPowers(runTime);
        }

        //This runs as the bot tries to reach the endpoint of the trajectory
        else if(state == States.ReachEndpoint) {
            Pose2d robotPose = drivetrain.getOdometryPose();
            Translation2d robotTranslation = robotPose.getTranslation();
            Pose2d goalPose = trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters;
            Translation2d goal = goalPose.getTranslation();

            
            //Check if the robot is within our preferred translation and angle errors
            //The command will also finish if the robot has been trying to get to the endpoint for too long
            if((robotTranslation.getDistance(goal) < PREFERRED_DISTANCE_ERROR 
                && Math.abs(robotPose.getRotation().getRadians() - goalPose.getRotation().getRadians()) < PREFERRED_ANGLE_ERROR)
                || runTime > trajectory.getTotalTimeSeconds() + timeToReachEndpoint) {
                
                finished = true;
                return;
            }

            sendMotorPowers(trajectory.getTotalTimeSeconds());
        }
    }

    private void sendMotorPowers(double timeIndexSeonds) {
        Trajectory.State goal = trajectory.sample(timeIndexSeonds);

        SmartDashboard.putNumber("robot pose x", drivetrain.getOdometryPose().getX());
        SmartDashboard.putNumber("robot pose y", drivetrain.getOdometryPose().getY());
        SmartDashboard.putNumber("robot pose theta", drivetrain.getOdometryPose().getRotation().getDegrees());

        SmartDashboard.putNumber("goal x", goal.poseMeters.getX());
        SmartDashboard.putNumber("goal y", goal.poseMeters.getY());
        SmartDashboard.putNumber("goal theta", goal.poseMeters.getRotation().getDegrees());


        ChassisSpeeds adjustedSpeeds = controller.calculate(drivetrain.getOdometryPose(), goal);
        DifferentialDriveWheelSpeeds wheelSpeeds = kDriveKinematics.toWheelSpeeds(adjustedSpeeds);

        drivetrain.tankDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.tankDrive(0, 0);
    }

    private static enum States {
        Following,
        ReachEndpoint
    }
}
