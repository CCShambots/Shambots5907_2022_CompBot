package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class MoveSpinnerCommand extends CommandBase{

    private Turret turret;
    private double target;
    
    public MoveSpinnerCommand(Turret turret, double target) {
        this.turret = turret;
        this.target = target;

        addRequirements(turret);
    }
    
    @Override
    public void execute() {
        System.out.println("Moving spinner");
    }

    @Override
    public void initialize() {
        turret.setSpinnerTarget(target);
    }

    @Override
    public boolean isFinished() {
        return !turret.isSpinnerBusy();
    }
    
}
