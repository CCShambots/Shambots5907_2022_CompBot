package frc.robot.util.statemachineframework;

import edu.wpi.first.util.sendable.SendableBuilder;

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
    
}
