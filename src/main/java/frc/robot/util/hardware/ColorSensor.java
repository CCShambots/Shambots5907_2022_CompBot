package frc.robot.util.hardware;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.Color;

public class ColorSensor implements Sendable{
    private DigitalInput pin1;
    private DigitalInput pin2;

    public ColorSensor(int port1, int port2) {
        this.pin1 = new DigitalInput(port1);
        this.pin2 = new DigitalInput(port2);
    }

    public Color getColor() {
        if(getPin1()) {
            if(getPin2()) return Color.NoBallDetected;
            else return Color.Red;
        } else {
            if(getPin2()) return Color.Blue;
            else return Color.SensorNotDetected;
        }

    }

    private boolean getPin1() {return pin1.get();} 
    private boolean getPin2() {return pin2.get();} 

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Digital Color Sensor");
        builder.addBooleanProperty("Pin 1 value", this::getPin1, null);
        builder.addBooleanProperty("Pin 2 value", this::getPin2, null);
        builder.addStringProperty("Color value", () -> getColor().name(), null);
    }

}