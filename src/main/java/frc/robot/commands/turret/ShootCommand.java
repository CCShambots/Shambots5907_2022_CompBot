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
    private long totalTime = 3000; //The total time the shooting command will run

    public ShootCommand(Conveyor conveyor) {
        this.conveyor = conveyor;

        // addRequirements(conveyor);
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

    }

    @Override
    public boolean isFinished() {
        return ballsShot;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stopAll();
        conveyor.clearTracker();
    }

    public static enum Amount { One, Two}
    
}
