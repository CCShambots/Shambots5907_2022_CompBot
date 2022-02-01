package frc.robot.commands.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase{
    //TODO: Make this actually do things :)

    private Intake intake;
    private Conveyor conveyor;

    private BooleanSupplier intakeSupplier; 
    private Boolean exhaustSupplier;
    private Boolean continueSupplier;
    
    public IntakeCommand(Intake intake, Conveyor conveyor, BooleanSupplier intakeSupplier, BooleanSupplier exhaustSupplier, BooleanSupplier continueSupplier) {
        this.intake = intake;
        this.conveyor = conveyor;

        addRequirements(intake, conveyor);
    }
    @Override
    public void initialize() {
        intake.lowerIntake();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        //TODO: Make this return something valid while balls are being handled
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        intake.raiseIntake();
    }
    
}
