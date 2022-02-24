package frc.robot.commands.intake;

import java.util.concurrent.CountDownLatch;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class ReleaseBallCommand extends CommandBase {
    private Intake intake;
    private Conveyor conveyor;
    private long endTime;
    private int count;
    private boolean finished;

    public ReleaseBallCommand(Intake i, Conveyor c, int count){
        intake = i;
        conveyor = c;

        addRequirements(intake, conveyor);

        finished = false;
        this.count = count;
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
