package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;


public class ReleaseBallCommand extends CommandBase {
    private Intake intake;
    private Conveyor conveyor;
    private long endTime;
    private boolean finished;

    //Messed up :'( ;-;
    @Deprecated
    public ReleaseBallCommand(Intake i, Conveyor c){
        intake = i;
        conveyor = c;

        addRequirements(intake, conveyor);

        finished = false;
    }
    
    @Override
    public void initialize() {
        intake.raiseIntake();
        conveyor.exhaustAll();
    }
    
    @Override
    public void execute() {
        if(conveyor.getNumberOfBalls() == 0
        ) {
            finished = true;
            endTime = System.currentTimeMillis();
        }

        if(finished) {
            if(System.currentTimeMillis()-endTime > 1500) end(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stopAll();
    }
}
