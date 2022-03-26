package frc.robot.commands.turret.limelight;

import frc.robot.subsystems.Turret;

public class AutonomousTargetCommand extends BasicTrackingCommand{
    
    public AutonomousTargetCommand(Turret turret) {
        super(turret);
    }

    @Override
    public boolean isComplete() {
        setTurretReadyFlag(isLockedIn());
        return isLockedIn();
    }

    @Override
    public void additionalCodeInInitialize() {}

    @Override
    public void additionalCodeInExecute() {}

    @Override
    public void additionalCodeInEnd() {}
    
}
