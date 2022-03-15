package frc.robot.util.ledAnimations;

public class SolidAnimation extends LEDAnimation{

    private RGB rgb;

    public SolidAnimation(RGB rgb) {
        this.rgb = rgb;
    }

    @Override
    public RGB sample(double timeIndex) {
        return rgb;
    }

    @Override
    public AnimationType getType() {
        return AnimationType.Solid;
    }
    
}
