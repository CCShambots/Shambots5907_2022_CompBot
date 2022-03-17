package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.lights.CanifierString;
import frc.robot.util.lights.RGB;
import frc.robot.util.lights.animations.AdvancedAnimation;
import frc.robot.util.lights.animations.BlinkingAnimation;
import frc.robot.util.lights.animations.LEDAnimation;
import frc.robot.util.lights.animations.LightState;

public class Lights extends SubsystemBase{
    CanifierString string = new CanifierString(Constants.Lights.CONTROLLER_ID);

    public Lights() {
        setAnimation(new AdvancedAnimation(new LightState(0, 0, 255, 2, 1), new LightState(255, 0, 0, 2, 1), new LightState(25, 255, 255, 2, 1)));
        setAnimation(new BlinkingAnimation(new RGB(255, 255, 255), new RGB(0, 0, 255), 3));
        setAnimation(AdvancedAnimation.acceleratingAnimation(new RGB(255, 255, 255), new RGB(0, 0, 255), 1.0, 2.5, .33, 20));
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
