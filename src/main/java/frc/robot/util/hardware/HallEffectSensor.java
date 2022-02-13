package frc.robot.util.hardware;

import edu.wpi.first.wpilibj.DigitalInput;

public class HallEffectSensor {
    private final DigitalInput sensor;

    public HallEffectSensor(int sensorPin) {
        sensor = new DigitalInput(sensorPin);
    }

    /**
     * @return Whether the sensor is activated or not
     */
    public boolean isActivated() {
        return !sensor.get();
    }
}

