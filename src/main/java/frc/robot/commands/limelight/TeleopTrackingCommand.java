package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Turret;

public class TeleopTrackingCommand extends BasicTrackingCommand{
    private Conveyor conveyor;

    public TeleopTrackingCommand(Turret turret, Conveyor conveyor) {
        super(turret);
        this.conveyor = conveyor;

        addRequirements(conveyor);
    }

    @Override
    public boolean isComplete() {
        //This will often be cancelled by itself, but it can also be cancelled
        return conveyor.getNumberOfBalls() == 0;
    }

    @Override
    public void additionalCodeInInitialize() {
        // turret.setFlywheelTarget(4300);
    }

    @Override
    public void additionalCodeInExecute() {
        //TODO: Remove this telemetry
        SmartDashboard.putBoolean("Teleop Tracking command is ready to shoot", isReady());
    }

    /**
     * 
     * @return true if the turret is locked into the target and the flywheel is spun up
     */
    public boolean isReady() {
        return 
            !turret.isFlywheelBusy() &&
            isLockedIn();
    }

    @Override
    public void additionalCodeInEnd() {
        turret.setFlywheelTarget(0);
        
    }
}
