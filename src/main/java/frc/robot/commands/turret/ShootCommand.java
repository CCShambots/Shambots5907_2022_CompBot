package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
    private long totalTime = 3; //The total time (in seconds) the shooting command will run
    private Timer timer;

    public ShootCommand(Conveyor conveyor) {
        this.conveyor = conveyor;

    }

    @Override
    public void initialize() {
        conveyor.intakeStage2();

        ballsShot = false;
        numberToShoot = amount == Amount.One ? 1 : 2;

        startingAmount = conveyor.getNumberOfBalls();
        if(numberToShoot > startingAmount) numberToShoot = startingAmount;

        if(conveyor.getNumberOfBalls() > 1) conveyor.intakeStage1();
        
        timer = new Timer();
        timer.start();
    }

    @Override
    public void execute() {
        if(timer.get() >= totalTime) {
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

        timer.stop();
    }

    public static enum Amount { One, Two}
    
}
