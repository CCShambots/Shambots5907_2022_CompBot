package frc.robot.commands.turret.limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.commands.turret.ShootCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.util.lights.RGB;
import frc.robot.util.lights.animations.SolidAnimation;

import static frc.robot.Constants.Turret.*;
import static frc.robot.Constants.*;

public class TeleopTrackingCommand extends BasicTrackingCommand{
    private Conveyor conveyor;
    private ShootCommand shootCommand = null;
    private boolean automatic;
    private Drivetrain drivetrain;

    public TeleopTrackingCommand(Drivetrain drivetrain, Turret turret, Conveyor conveyor, boolean automatic) {
        super(turret);
        this.conveyor = conveyor;
        this.automatic = automatic;
        this.drivetrain = drivetrain;

        addRequirements(turret, conveyor);
    }

    public TeleopTrackingCommand(Drivetrain drivetrain, Turret turret, Conveyor conveyor) {
        this(drivetrain, turret, conveyor, false);
    }

    @Override
    public boolean isComplete() {
        boolean shouldEndAutomatic = false;
        if(automatic) {
            shouldEndAutomatic = drivetrain.getOdometryPoseFeet().getTranslation().getDistance(goalPos) > AUTOMATIC_START_DISTANCE;
        }

        //This will often be cancelled by itself, but it can also be cancelled by the drivers 
        return 
            conveyor.getNumberOfBalls() == 0 || 
            turret.getShouldEndTargeting() ||
            shouldEndAutomatic;
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

        if(conveyor.getNumberOfBalls() == 0) turret.setShouldEndTargeting(false);
    }
}
