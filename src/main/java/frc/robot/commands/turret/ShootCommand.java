package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Conveyor;

/**
 * This command moves the conveyor until the set number of balls have exited the robot 
 */
public class ShootCommand extends CommandBase{

    private Conveyor conveyor;
    private Amount amount;
    private int numberToShoot;
    private int startingAmount;

    private boolean ballsShot = false;
    private long startTime = 0;
    private long totalTime = 3000;

    private RobotContainer robotContainer;

    public ShootCommand(Conveyor conveyor, Amount amount, RobotContainer robotTracker) {
        this.conveyor = conveyor;

        // addRequirements(conveyor);

        this.amount = amount;
        this.robotContainer = robotTracker;
    }

    public ShootCommand(Conveyor conveyor, Amount amount) {
        this(conveyor, amount, null);
    }

    @Override
    public void initialize() {
        conveyor.intakeStage2();

        ballsShot = false;
        numberToShoot = amount == Amount.One ? 1 : 2;

        startingAmount = conveyor.getNumberOfBalls();
        if(numberToShoot > startingAmount) numberToShoot = startingAmount;

        if(conveyor.getNumberOfBalls() > 1) conveyor.intakeStage1();
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        if(System.currentTimeMillis() - startTime >= totalTime) {
            ballsShot = true;
        }

        // if(conveyor.getNumberOfBalls() == 1 && conveyor.getBall1Pos() == BallPosition.Stage2) {
        //     ballAdvanced = true;
        // }
    }

    @Override
    public boolean isFinished() {
        return ballsShot;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stopAll();
        conveyor.clearTracker();
        if(robotContainer != null) robotContainer.toggleLimelightTargeting();
    }

    public static enum Amount { One, Two}
    
}
