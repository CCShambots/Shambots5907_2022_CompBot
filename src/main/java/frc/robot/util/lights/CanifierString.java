package frc.robot.util.lights;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.lights.animations.LEDAnimation;
import frc.robot.util.lights.animations.SolidAnimation;
import frc.robot.util.lights.animations.LEDAnimation.AnimationType;

public class CanifierString implements Sendable{

    private CANifier controller;
    private LEDAnimation animation = new SolidAnimation(new RGB(255, 255, 255));

    private double[] prevSample = {0,0,0};
    private double[] currentSample;
    private Timer animationTimer;
    

    public CanifierString(int id) {
        controller = new CANifier(id);
        animationTimer = new Timer();

    }

    public void setAnimation(LEDAnimation animation) {
        //Only restart the animation if it is different from the current animation
        if(!this.animation.equals(animation)) {
            this.animation = animation;
            if(animation.getType() == AnimationType.Variable) {
                //If the animation is variable, restart the timer and send commands to the lights that way
                animationTimer.stop();
                animationTimer.reset();
                animationTimer.start();
            } else if(animation.getType() == AnimationType.Solid) {
                //If the animation is just solid, only send a command to the lights once 
                setLights(animation.sample(0).toPercentage());
            }
        }
    }

    public void periodic() {
        //Only update LED colors if the animation is variable
        if(animation.getType() == AnimationType.Variable) {
            RGB rgb = animation.sample(animationTimer.get());
            currentSample = rgb.toPercentage();
            
            if(!sameState(prevSample, currentSample)) {
                setLights(currentSample);
            }

            prevSample = currentSample;
        }
    }

    private void setLights(double[] percentages) {
        controller.setLEDOutput(percentages[0], LEDChannel.LEDChannelB);
        controller.setLEDOutput(percentages[1], LEDChannel.LEDChannelA);
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

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("CANifier");
        builder.addDoubleProperty("r", () -> currentSample[0] * 255, null);
        builder.addDoubleProperty("g", () -> currentSample[1] * 255, null);
        builder.addDoubleProperty("b", () -> currentSample[2] * 255, null);
    }
}
