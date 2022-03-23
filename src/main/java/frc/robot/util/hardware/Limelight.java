package frc.robot.util.hardware;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight{

    /**Limelight subsystem that returns all the information we use for targetting */
    public Limelight() {}
    
    /**
     * @return The limelight's data table (used to access values from the pipeline)
     */
    private NetworkTable getLimeLightTable() {
        return NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**
     * @return true if the limelight has identified a valid target
     */
    public boolean hasTarget() {
        return getLimeLightTable().getEntry("tv").getDouble(0) == 1;
    }

    /**
     * Note: The returned values are raw from the limelight. They are negated (if needed) in the turret subsystem
     * @return how far the detected target is from the center of the limelight frame in degrees
     */
    public Translation2d targetOffset() {
        return new Translation2d(
            -getLimeLightTable().getEntry("tx").getDouble(0),
            getLimeLightTable().getEntry("ty").getDouble(0)
        );
    }

    /**
     * Turns on the limelight
     */
    public void setOn() {
        getLimeLightTable().getEntry("ledMode").setNumber(3);
    }

    /**
     * Turns off the limelight
     */
    public void setOff() {
        getLimeLightTable().getEntry("ledMode").setNumber(1);
    }

    /**
     * @return the latency of the limelight (in ms)
     */
    public double getLatency() {
        return getLimeLightTable().getEntry("tl").getDouble(0);
    }
}
