package frc.robot.util.ledAnimations;

public abstract class LEDAnimation {
    /**
     * @param timeIndex the time index (in seconds) since the animation has started
     * @return percentage values for each channel
     */
    public abstract RGB sample(double timeIndex);
    public abstract AnimationType getType();


    public static enum AnimationType {
        Solid, Variable
    }
}
