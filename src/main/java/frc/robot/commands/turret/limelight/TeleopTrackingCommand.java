package frc.robot.commands.turret.limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.commands.turret.ShootCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Turret;
import frc.robot.util.lights.RGB;
import frc.robot.util.lights.animations.BlinkingAnimation;
import frc.robot.util.lights.animations.SolidAnimation;

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

        boolean ready = !turret.isFlywheelBusy() && isLockedIn();

        if(ready) RobotContainer.lights.setAnimation(new SolidAnimation(new RGB(0, 255, 255)));

        return ready;
    }

    @Override
    public void additionalCodeInEnd() {
        turret.setFlywheelTarget(0);
        turret.setShouldEndTargeting(false);

        switch (conveyor.getNumberOfBalls()) {
            case 0:
                RobotContainer.lights.setAnimation(new SolidAnimation(new RGB(0, 0, 0)));
                break;
            case 1:
                RobotContainer.lights.setAnimation(new BlinkingAnimation(new RGB(0, 0, 255), new RGB(255, 255, 255), 3));
                break;
            case 2:
                RobotContainer.lights.setAnimation(new SolidAnimation(new RGB(0, 0, 255)));
                break;
        }
    }
}
