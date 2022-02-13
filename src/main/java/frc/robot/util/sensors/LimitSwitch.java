package frc.robot.util.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitch {

    private DigitalInput input;

    public LimitSwitch(int port) {
        input = new DigitalInput(port);
    }

    public boolean isActivated() {return input.get();}
    
}
