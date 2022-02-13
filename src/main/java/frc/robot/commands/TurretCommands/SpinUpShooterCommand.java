package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

/**
 * Simple command that will run until the flywheel is at the target velocity
*/
public class SpinUpShooterCommand extends CommandBase{
    private Turret turret;
    private double targetRPM ;

    public SpinUpShooterCommand(Turret turret, double targetRPM) {
        this.turret = turret;
        this.targetRPM = targetRPM;
    }

    @Override
    public void initialize() {
        turret.setFlywheelTarget(targetRPM);
    }

    @Override
    public boolean isFinished() {
        return !turret.isFlywheelBusy();
    }
    
}
