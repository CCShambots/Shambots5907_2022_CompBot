package frc.robot.commands.turret.limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.turret.ShootCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Turret;

import static frc.robot.Constants.Turret.*;

public class TeleopTrackingCommand extends BasicTrackingCommand{
    private Conveyor conveyor;
    private ShootCommand shootCommand = null;

    public TeleopTrackingCommand(Turret turret, Conveyor conveyor) {
        super(turret);
        this.conveyor = conveyor;

        addRequirements(turret, conveyor);
    }

    @Override
    public boolean isComplete() {
        //This will often be cancelled by itself, but it can also be cancelled
        return conveyor.getNumberOfBalls() == 0 || turret.getShouldEndTargeting();
    }

    @Override
    public void additionalCodeInInitialize() {
        turret.setShouldEndTargeting(false);
        turret.setFlywheelTarget(FLYWHEEL_TARGET_RPM);

        shootCommand = null;
    }

    @Override
    public void additionalCodeInExecute() {
        //TODO: Remove this telemetry
        SmartDashboard.putBoolean("Teleop Tracking command is ready to shoot", isReady());

        if(turret.getShouldShoot() 
            && shootCommand == null
            && isReady()) 
        {
            shootCommand = new ShootCommand(conveyor);
            shootCommand.schedule();
        }
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
        turret.setShouldEndTargeting(false);
    }
}
