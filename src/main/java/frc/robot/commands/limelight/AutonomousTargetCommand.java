package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Turret;

public class AutonomousTargetCommand extends BasicTrackingCommand{
    
    public AutonomousTargetCommand(Turret turret) {
        super(turret);
    }

    @Override
    public boolean isComplete() {

        SmartDashboard.putBoolean("Is locked in", isLockedIn());
        return isLockedIn();
    }

    @Override
    public void additionalCodeInInitialize() {}

    @Override
    public void additionalCodeInExecute() {}

    @Override
    public void additionalCodeInEnd() {}
    
}
