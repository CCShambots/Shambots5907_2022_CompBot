package frc.robot.util.hardware;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.robot.Constants;

public class LidarSensor {
    
    private final DutyCycle sensor;
    private final DigitalOutput triggerPin;

    /**
     * Creates a new LiDAR sensor
     * @param monitorPin the digital pin for the monitor output (reading values from the LiDAR)
     * @param triggerPin the digital pin for the LiDAR trigger (turning it on or off)
     */
    public LidarSensor(int monitorPin, int triggerPin) {
        this.sensor = new DutyCycle(new DigitalInput(monitorPin));
        this.triggerPin = new DigitalOutput(triggerPin);
    }


    /**
     * @return The Lidar's output directly from the sensor (value 0-1)
     */
    public double getOutput() {
        return sensor.getOutput();
    }

    /**
     * @return The lidar's output in meters (multiplies the raw output)
     */
    public double getOutputMeters() {
        return getOutput() * Constants.Lidar.LIDAR_VALUE_IN_METERS;
    }

    /**
     * Sets the trigger pin to either active or inactive
     * @param state true = Active    false = Inactive
     */
    public void setOn(boolean state) {
        triggerPin.set(!state);
    }




}
