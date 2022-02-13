package frc.robot.commands.TurretCommands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

import static frc.robot.Constants.Turret.*;

public class ZeroSpinnerCommand extends CommandBase{    
    Turret turret;
    private double approximateStartingAngle;
    Constraints originalConstraints = new TrapezoidProfile.Constraints(SPINNER_MAX_VEL, SPINNER_MAX_ACCEL);

    boolean finished = false;

    /**
     * @param turret the turret subsystem
     * @param approximateStartingAngle the angle at which the spinner approximately starts (for zeroing out)
     */
    public ZeroSpinnerCommand(Turret turret, double approximateStartingAngle) {
        this.turret = turret;
        this.approximateStartingAngle = approximateStartingAngle;
    }

    @Override
    public void initialize() {
        turret.resetSpinnerAngle(approximateStartingAngle);
        turret.setSpinnerConstraints(new TrapezoidProfile.Constraints(45, SPINNER_MAX_ACCEL));
        turret.setSpinnerTarget(-approximateStartingAngle);
    }

    @Override
    public void execute() {
        if(turret.getCenterHallEffectSensor().isActivated()) {
            turret.resetSpinnerAngle(0);
            turret.setSpinnerTarget(0);
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }


    @Override
    public void end(boolean interrupted) {
        turret.setSpinnerConstraints(originalConstraints);
    }

}
