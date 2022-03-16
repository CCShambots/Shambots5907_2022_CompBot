package frc.robot.util.lights;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.util.lights.animations.LEDAnimation;
import frc.robot.util.lights.animations.SolidAnimation;
import frc.robot.util.lights.animations.LEDAnimation.AnimationType;

public class CanifierString{

    private CANifier controller = new CANifier(Constants.Lights.CONTROLLER_ID);
    private LEDAnimation animation = new SolidAnimation(new RGB(255, 255, 255));

    private double[] prevSample;
    private double[] currentSample;
    private Timer animationTimer;

    public CanifierString() {
        animationTimer = new Timer();
    }

    public void setAnimation(LEDAnimation animation) {
        this.animation = animation;
        if(animation.getType() == AnimationType.Variable) {
            animationTimer.stop();
            animationTimer.reset();
            animationTimer.start();
        } else if(animation.getType() == AnimationType.Solid) {
            setLights(animation.sample(0).toPercentage());
        }
    }

    public void periodic() {
        if(animation.getType() == AnimationType.Variable) {
            currentSample = animation.sample(animationTimer.get()).toPercentage();
            
            if(!sameState(prevSample, currentSample)) {
                setLights(currentSample);
            }

            prevSample = currentSample;
        }
    }

    private void setLights(double[] percentages) {

        //TODO: Remove
        System.out.println("Sending CAN data to lights");

        controller.setLEDOutput(percentages[0], LEDChannel.LEDChannelA);
        controller.setLEDOutput(percentages[1], LEDChannel.LEDChannelB);
        controller.setLEDOutput(percentages[2], LEDChannel.LEDChannelC);
    }

    /**
     * 
     * @param color1 three element array of percentages for RGB channels
     * @param color2 three element array of percentages for RGB channels
     * @return
     */
    private boolean sameState(double[] color1, double[] color2) {
        if(color1[0] != color2[0]) return false;
        if(color1[1] != color2[1]) return false;
        if(color1[2] != color2[2]) return false;
        return true;
    }
    
}
