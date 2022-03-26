package frc.robot.commands.lights;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Turret;
import static frc.robot.Constants.Lights.*;

public class DefaultLightCommand extends CommandBase{


    private Lights lights;
    private Conveyor conveyor;
    private Turret turret;

    public DefaultLightCommand(Lights lights, Drivetrain drivetrain, Intake intake, Conveyor conveyor, Turret turret) {
        this.lights = lights;
        this.conveyor = conveyor;
        this.turret = turret;

        addRequirements(lights);
    }

    @Override
    public void initialize() {
        lights.setAnimation(EMPTY_ANIMATION);
    }

    @Override
    public void execute() {
        if(conveyor.isTrackerError()) {
            lights.setAnimation(ERROR_ANIMATION);
        }else {
            if(turret.getIsReadyToShoot()) {
                lights.setAnimation(LOCKED_IN_ANIMATION);
            } else {
                if(conveyor.getNumberOfBalls() == 2) {
                    lights.setAnimation(TWO_BALL_ANIMATION);
                } else if(conveyor.getNumberOfBalls() == 1) {
                    lights.setAnimation(ONE_BALL_ANIMATION);
                }else {
                    lights.setAnimation(EMPTY_ANIMATION);
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        lights.setAnimation(DEFAULT_ANIMAION);
    }
}
