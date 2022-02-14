package frc.robot.commands.limelight;

import frc.robot.subsystems.Turret;

public class AutonomousTargetCommand extends BasicTrackingCommand{
    
    public AutonomousTargetCommand(Turret turret) {
        super(turret);
    }

    @Override
    public boolean isComplete() {
        return isLockedIn();
    }

    @Override
    public void additionalCodeInInitialize() {}

    @Override
    public void additionalCodeInExecute() {}

    @Override
    public void additionalCodeInEnd() {}
    
}
