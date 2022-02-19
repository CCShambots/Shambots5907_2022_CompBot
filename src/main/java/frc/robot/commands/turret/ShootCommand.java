package frc.robot.commands.turret;

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

    private boolean finished = false;

    public ShootCommand(Conveyor conveyor, Amount amount) {
        this.conveyor = conveyor;

        addRequirements(conveyor);

        this.amount = amount;
    }

    @Override
    public void initialize() {
        conveyor.intakeStage2();

        finished = false;
        numberToShoot = amount == Amount.One ? 1 : 2;

        startingAmount = conveyor.getNumberOfBalls();
        if(numberToShoot > startingAmount) numberToShoot = startingAmount;

        if(conveyor.getNumberOfBalls() > 1 &&  numberToShoot == 2) conveyor.intakeStage1();
    }

    @Override
    public void execute() {
        if(conveyor.getNumberOfBalls() <= startingAmount - numberToShoot) {
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stopAll();
    }

    public static enum Amount { One, Two}
    
}
