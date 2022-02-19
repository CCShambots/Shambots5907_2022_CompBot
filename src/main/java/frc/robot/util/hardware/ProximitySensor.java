package frc.robot.util.hardware;

import edu.wpi.first.wpilibj.DigitalInput;

public class ProximitySensor {
    private DigitalInput input;

    public ProximitySensor(int pin) {
        input = new DigitalInput(pin);
    }

    public boolean isActivated() {
        return !input.get();
    }
}
