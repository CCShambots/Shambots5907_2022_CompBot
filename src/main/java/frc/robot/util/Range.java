package frc.robot.util;

public class Range {
    private double lowerLimit;
    private double upperLimit;

    /**
     * A range between two numbers. Note, bounds 1 and 2 do not need to be in order; they will be sorted automatically
     * @param bound1 arbitrary double value
     * @param bound2 arbutrary double value
     */
    public Range(double bound1, double bound2) {
        lowerLimit = Math.min(bound1, bound2);
        upperLimit = Math.max(bound1, bound2);
    }

    /**
     * 
     * @param input the number to be compared
     * @return true if the number is within the two bounds
     */
    public boolean inRange(double input) {
        return input >= lowerLimit && input <= upperLimit;
    }

}
