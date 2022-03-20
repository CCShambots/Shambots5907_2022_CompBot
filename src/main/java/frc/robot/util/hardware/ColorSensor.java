package frc.robot.util.hardware;

import edu.wpi.first.wpilibj.PWM;
import frc.robot.Constants.Color;

public class ColorSensor{
    private PWM pin;

    public ColorSensor(int pwmPort) {
        this.pin = new PWM(pwmPort);

        // pin.getRaw()
    }


    //TODO: Actually get color sensor values
    public Color getColor() {
        return Color.Red;
    }
}