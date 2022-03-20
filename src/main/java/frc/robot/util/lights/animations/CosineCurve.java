package frc.robot.util.lights.animations;

public class CosineCurve {
    private double a;
    private double b;
    private double c;
    private double d;

    public CosineCurve(double a, double b, double c, double d) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }

    public double get(double x) {
        return a*Math.cos(b*(x + c)) + d;
    }
}
