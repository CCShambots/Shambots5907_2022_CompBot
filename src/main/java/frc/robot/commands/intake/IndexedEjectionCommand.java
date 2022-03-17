package frc.robot.commands.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

public class IndexedEjectionCommand extends CommandBase{

    private Conveyor conveyor;
    private Intake intake;
    private BooleanSupplier ejectAdditionalBallSupplier;
    private int startingBalls;

    private int numToEject = 1;

    public IndexedEjectionCommand(Conveyor conveyor, Intake intake, BooleanSupplier supplier) {
        this.conveyor = conveyor;
        this.intake = intake;
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
    }

    @Override
    public void execute() {
        if(ejectAdditionalBallSupplier.getAsBoolean() && numToEject == 1) {
            conveyor.exhaustStage2();
            numToEject++;
        }


    }

    @Override
    public boolean isFinished() {

    }

    @Override
    public void end(boolean interrupted) {
        conveyor.setEjecting(false);
    }
    
}
