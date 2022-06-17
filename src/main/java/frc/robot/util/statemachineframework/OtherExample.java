package frc.robot.util.statemachineframework;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.util.Shambots5907_SMF.StatedSubsystem;

public class OtherExample extends StatedSubsystem<OtherExample.State>{

    public OtherExample() {
        super(OtherExample.State.class);
    }

    public enum State {
        Undetermined, Idle
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "other example";
    }

    @Override
    public void additionalSendableData(SendableBuilder builder) {
        // TODO Auto-generated method stub
        
    }
    
}
