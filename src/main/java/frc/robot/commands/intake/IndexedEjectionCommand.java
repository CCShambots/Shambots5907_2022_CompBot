package frc.robot.commands.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.Conveyor.*;

public class IndexedEjectionCommand extends CommandBase{

    private Conveyor conveyor;

    private BooleanSupplier ejectAdditionalBallSupplier;
    private boolean prevButtonValue;

    private int startingBalls;
    private int numToEject = 1;

    private Timer timer = new Timer();
    private boolean ballLeftTracker;

    public IndexedEjectionCommand(Conveyor conveyor, Intake intake, BooleanSupplier supplier) {
        this.conveyor = conveyor;
        this.ejectAdditionalBallSupplier = supplier;
    
        addRequirements(conveyor, intake);
    }

    @Override
    public void initialize() {
        startingBalls = conveyor.getNumberOfBalls();
        
        conveyor.exhaustStage1();
        //Run the second stage if there is only one ball (it would already be in the second stage)
        if(conveyor.getNumberOfBalls() == 1) conveyor.exhaustStage2(); 
        conveyor.setEjecting(true);

        prevButtonValue = ejectAdditionalBallSupplier.getAsBoolean();

        timer.reset();
        ballLeftTracker = false;
    }

    @Override
    public void execute() {
        boolean currentButtonValue = ejectAdditionalBallSupplier.getAsBoolean();

        if(currentButtonValue && prevButtonValue != currentButtonValue && numToEject == 1 && startingBalls == 2) {
            conveyor.exhaustStage2();
            numToEject++;
        }

        prevButtonValue = currentButtonValue;

        if(conveyor.getNumberOfBalls() == startingBalls - numToEject && !ballLeftTracker) {
            timer.start();
        }

        if(conveyor.getNumberOfBalls() == startingBalls - numToEject) ballLeftTracker = true;
    }

    @Override
    public boolean isFinished() {
        return timer.get() > EJECTION_DELAY;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        conveyor.setEjecting(false);
        conveyor.stopAll();
    }
    
}
