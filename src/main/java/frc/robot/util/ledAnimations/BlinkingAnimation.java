package frc.robot.util.ledAnimations;

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
        double remainder = timeIndex % (2 * timeBetweenSwitch);

        if(remainder > timeBetweenSwitch) {
            return color1;
        } else return color2;                
    }

    @Override
    public AnimationType getType() {
        return AnimationType.Variable;
    }
    
}
