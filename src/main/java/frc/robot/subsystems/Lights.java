package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.lights.CanifierString;
import frc.robot.util.lights.animations.LEDAnimation;

import static frc.robot.Constants.Lights.*;

public class Lights extends SubsystemBase{
    CanifierString string = new CanifierString(Constants.Lights.CONTROLLER_ID);

    public Lights() {
        setAnimation(DEFAULT_ANIMAION);
    }

    public void setAnimation(LEDAnimation animation) {
        string.setAnimation(animation);
    }

    @Override
    public void periodic() {
        string.periodic();

        SmartDashboard.putData("Lights", string);
    }

    
}
