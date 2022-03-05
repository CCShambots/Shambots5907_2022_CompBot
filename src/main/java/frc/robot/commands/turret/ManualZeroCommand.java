package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Turret.Direction;

import static frc.robot.Constants.Turret.*;

public class ManualZeroCommand extends CommandBase{ 
    private Turret turret;
    private Direction direction;
    private boolean finished = false;

    public ManualZeroCommand(Turret turret, Direction direction) {
        this.turret = turret;
        this.direction = direction;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setSpinnerConstraints(turret.getSlowSpinnerConstraints());

        double target = direction == Direction.Clockwise ? SPINNER_CLOCKWISE_LIMIT : SPINNER_COUNTERCLOCKWISE_LIMIT;

        turret.setSpinnerTarget(target);
    }

    @Override
    public void execute() {
        if(turret.isClockwiseHallEffectActivated()) {
            turret.resetSpinnerAngle(CLOCKWISE_SENSOR_ANGLE);
            finished = true;
        }

        else if(turret.isCounterclockwiseHallEffectActivated()) {
            turret.resetSpinnerAngle(COUNTERCLOCKWISE_SENSOR_ANGLE);
            finished = true;
        }

        if(Math.abs(turret.getSpinnerAngle() - SPINNER_CLOCKWISE_LIMIT) < 2 || Math.abs(turret.getSpinnerAngle() - SPINNER_COUNTERCLOCKWISE_LIMIT) < 2) {
            end(true);
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        if(!interrupted) {
            turret.setSpinnerConstraints(turret.getOriginalSpinnerConstraints());
            turret.setKnowsLocation(true);
        }
        turret.setSpinnerTarget(0);
    }
}
