package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.util.lights.RGB;
import frc.robot.util.lights.animations.SolidAnimation;

public class HardEjectCommand extends CommandBase{

    Conveyor conveyor;
    Intake intake;
    Timer timer;
    double time;

    /**
     * 
     * @param conveyor subsystem
     * @param intake subsystem
     * @param time time (in seconds) to exhaust the conveyor
     */
    public HardEjectCommand(Conveyor conveyor, Intake intake, double time) {
        this.conveyor = conveyor;
        this.intake = intake;

        this.time = time;

        this.addRequirements(conveyor, intake);
    }

    @Override
    public void initialize() {
        timer = new Timer();

        timer.start();

        conveyor.setEjecting(true);

        conveyor.exhaustAll();
        conveyor.setTrackerDisabled(true);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > time;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stopAll();
        conveyor.clearTracker();
        conveyor.setTrackerDisabled(false);

        timer.stop();
        
        conveyor.setEjecting(false);

        RobotContainer.lights.setAnimation(new SolidAnimation(new RGB(0, 0, 0)));
    }
}
