package frc.robot.util.lights.animations;


import frc.robot.util.lights.RGB;

public class BlinkingAnimation extends LEDAnimation{

    private RGB color1;
    private RGB color2;
    private double timeBetweenSwitch;

    /**
     * 
     * @param color1
     * @param color2
     * @param frequency The frequency to blink (in Hz)
     */
    public BlinkingAnimation(RGB color1, RGB color2, double frequency) {
        this.color1 = color1;
        this.color2 = color2;
        this.timeBetweenSwitch = 1.0 / frequency;
    }

    @Override
    public RGB sample(double timeIndex) {
        double remainder = timeIndex % (timeBetweenSwitch);

        if(remainder > (timeBetweenSwitch / 2.0)) {
            return color2;
        } else return color1;

    }

    @Override
    public AnimationType getType() {
        return AnimationType.Variable;
    }
    
}
