package frc.robot.util.lights.animations;


import java.util.ArrayList;
import java.util.List;

import frc.robot.util.lights.RGB;

public class AdvancedAnimation extends LEDAnimation{
    private List<AnimationState> states;
    private double totalTime;

    public AdvancedAnimation(LightState... lState) {
        List<LightState> lightStates = List.of(lState);

        states = new ArrayList<>();
        double totalTime = 0;
        for(int i = 0; i<lightStates.size(); i++) {
            LightState s = lightStates.get(i);
            LightState next = lightStates.size() >= i+2 ? lightStates.get(i+1) : lightStates.get(0);
            states.add(new AnimationState(s, totalTime, next));
            totalTime += s.getHoldTime();
            totalTime += s.getTransitionTime();
        }

        this.totalTime = totalTime;
    }


    @Override
    public RGB sample(double timeIndex) {
        double timeInCycle = timeIndex % totalTime;

        AnimationState currentState = states.stream().filter((s) -> timeInCycle >= s.startTime && timeInCycle <= (s.startTime + s.totalDuration)).findAny().get();

        return currentState.getRGB(timeInCycle - currentState.startTime);
    }

    @Override
    public AnimationType getType() {
        return AnimationType.Variable;
    }

    class AnimationState {
        RGB color;
        double holdTime;
        double transitionTime;
        double startTime;
        double totalDuration;
        CosineCurve rAnimation;
        CosineCurve gAnimation;
        CosineCurve bAnimation;

        AnimationState(LightState state, double startTime, LightState next) {
            this.color = state.getColor();
            this.holdTime = state.getHoldTime();
            this.transitionTime = state.getTransitionTime();
            this.startTime = startTime;
            this.totalDuration = holdTime+transitionTime;

            double b = (Math.PI) / transitionTime;
            double c = -(holdTime);

            double rA = (color.getR() - next.getColor().getR())/2.0;
            double rD = color.getR() + (.5 * (next.getColor().getR()-color.getR()));

            double gA = (color.getG() - next.getColor().getG())/2.0;
            double gD = color.getG() + (.5 * (next.getColor().getG()-color.getG()));

            double bA = (color.getB() - next.getColor().getB())/2.0;
            double bD = color.getB() + (.5 * (next.getColor().getB()-color.getB()));

            rAnimation = new CosineCurve(rA, b, c, rD);
            gAnimation = new CosineCurve(gA, b, c, gD);
            bAnimation = new CosineCurve(bA, b, c, bD);
            
        }


        RGB getRGB(double timeInState) {
            if(timeInState <= holdTime) {
                return color;
            }
            RGB rgb = new RGB((int) rAnimation.get(timeInState), (int) gAnimation.get(timeInState), (int) bAnimation.get(timeInState));
        
            return rgb;
        }
    }
    
    /**
     * 
     * @param color1 RGB of the first color to use in the sequence
     * @param color2 RGB of the second color to use in the sequence
     * @param initialHoldTime How long the animation should hold at each color to begin with (in seconds)
     * @param initialTransitionTime How long the animation should transition for at the beginning of the animation (seconds)
     * @param acceleration What percentage (0.0-1.0) the animatino should speed up by each iteration
     * @param iterations How many times the animatino should speed up
     * @return
     */
    public static AdvancedAnimation acceleratingAnimation(RGB color1, RGB color2, double initialHoldTime, double initialTransitionTime, double acceleration, int iterations) {
        List<LightState> states = new ArrayList<>();
        double currentHoldTime = initialHoldTime;
        double currentTransitionTime = initialTransitionTime;

        for(int i = 0; i<=iterations; i++) {
            states.add(new LightState(color1, currentHoldTime, currentTransitionTime));
            states.add(new LightState(color2, currentHoldTime, currentTransitionTime));
    
            currentHoldTime *= 1-acceleration;
            currentTransitionTime *= 1-acceleration;
        }
    
        LightState[] stateArray = new LightState[states.size()];

        for(int i=0; i<states.size(); i++) {
            stateArray[i] = states.get(i);
        }

        return new AdvancedAnimation(stateArray);
    }
}
