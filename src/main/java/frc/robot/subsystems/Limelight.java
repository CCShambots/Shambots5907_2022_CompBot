package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{

    /**Limelight subsystem that returns all the information we use for targetting */
    public Limelight() {}
    
    /**The data table used by the limelight */
    private NetworkTable getLimeLightTable() {
        return NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**Returns whether a target is seen by the limelight */
    public boolean hasTarget() {
        return getLimeLightTable().getEntry("tv").getDouble(0) == 1;
    }

    /**Returns how far the detected target is from the center of the limelight frame in degrees */
    public Translation2d targetOffset() {
        return new Translation2d(
            getLimeLightTable().getEntry("tx").getDouble(0),
            getLimeLightTable().getEntry("ty").getDouble(0)
        );
    }

    /**Turns the limelight's light on */
    public void setOn() {
        getLimeLightTable().getEntry("ledMode").setNumber(3);
    }

    /**Turns the limelight's light off */
    public void setOff() {
        getLimeLightTable().getEntry("ledMode").setNumber(1);
    }
    
}
