package frc.robot.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class FakeGyro implements Gyro, Sendable{

    DoubleSupplier angleSupplier;

    public FakeGyro(DoubleSupplier angSupplier) {
        angleSupplier = angSupplier;
    }


    @Override
    public void close() throws Exception {
        
    }

    @Override
    public void calibrate() {
        
    }

    @Override
    public void reset() {
        
    }

    /**
     * Converts the angle of the turret (180 to -180) to the gyro coordinates
     */
    @Override
    public double getAngle() {
        double turretValue = angleSupplier.getAsDouble();
        return turretValue > 0 ?
            360 - turretValue :
            -turretValue;
    }

    @Override
    public double getRate() {
        return 0;
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", this::getAngle, null);
    }
    
}
