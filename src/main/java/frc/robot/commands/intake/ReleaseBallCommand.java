package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class ReleaseBallCommand extends CommandBase {
    private Intake intake;
    private Conveyor conveyor;

    public ReleaseBallCommand(Intake i, Conveyor c){
        intake = i;
        conveyor = c;

        addRequirements(intake, conveyor);
    }
    
    @Override
    public void initialize() {
        intake.raiseIntake();
        conveyor.exhaustAll();
    }
    
    @Override
    public void execute() {
        if(!conveyor.hasBalls()){
            end(false);
        }
    
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stopAll();
    }
}
