package frc.robot.util.lights.animations;

import frc.robot.util.lights.RGB;

public class LightState {
    private RGB color;
    private double holdTime;
    private double transitionTime;

    /**
     * 
     * @param color RGB values for the desired color of the lights
     * @param holdTime time (in seconds) to hold the set color before beginning the transition
     * @param transitionTime time (in seconds) that it should take to fully get to the next state
     */
    public LightState(RGB color, double holdTime, double transitionTime) {
        this.color = color;
        this.holdTime = holdTime;
        this.transitionTime = transitionTime;
    }

    public LightState(int r, int g, int b, double holdTime, double transitionTime) {
        this(new RGB(r, g, b), holdTime, transitionTime);
    }

    public RGB getColor() {return color;}
    public double getHoldTime() {return holdTime;}
    public double getTransitionTime() {return transitionTime;}
}
