package frc.robot.commands.turret.limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.commands.turret.ShootCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

import static frc.robot.Constants.Turret.*;

public class TeleopTrackingCommand extends BasicTrackingCommand{
    private Conveyor conveyor;
    private ShootCommand shootCommand = null;
    private Intake intake;

    public TeleopTrackingCommand(Turret turret, Conveyor conveyor, Intake intake) {
        super(turret);
        this.conveyor = conveyor;
        this.intake = intake;

        //We include the intake so that an intake command can't be started while targeting (it should cancel the targeting)
        addRequirements(turret, conveyor);
    }

    @Override
    public boolean isComplete() {
        //This will often be cancelled by itself, but it can also be cancelled
        return conveyor.getNumberOfBalls() == 0;
    }

    @Override
    public void additionalCodeInInitialize() {
        turret.setFlywheelTarget(FLYWHEEL_TARGET_RPM);

        shootCommand = null;
    }

    @Override
    public void additionalCodeInExecute() {
        //TODO: Remove this telemetry
        SmartDashboard.putBoolean("Teleop Tracking command is ready to shoot", isReady());

        if(turret.getShouldShoot() 
            && shootCommand == null 
            && intake.getCurrentCommand() == null 
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
    }
}
