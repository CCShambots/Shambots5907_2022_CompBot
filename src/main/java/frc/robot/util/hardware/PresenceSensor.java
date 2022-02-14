package frc.robot.util.hardware;

import edu.wpi.first.wpilibj.DigitalInput;

public class PresenceSensor {
    private DigitalInput input;

    public PresenceSensor(int pin) {
        input = new DigitalInput(pin);
    }

    public boolean isActivated() {
        return !input.get();
    }
}
