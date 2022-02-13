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
        // TODO Auto-generated method stub
        
    }

    @Override
    public void calibrate() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public double getAngle() {
        // TODO Auto-generated method stub
        return angleSupplier.getAsDouble();
    }

    @Override
    public double getRate() {
        // TODO Auto-generated method stub
        return 0;
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Fake Gyro");
        builder.addDoubleProperty("Value", this::getAngle, null);
    }
    
}
