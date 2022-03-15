package frc.robot.util.ledAnimations;

public class RGB {
    private double r;
    private double g;
    private double b;

    /**
     * 
     * @param r red channel (0-255)
     * @param g green channel (0-255)
     * @param b blue channel (0-255)
     */
    public RGB(double r, double g, double b) {
        this.r = r;
        this.g = g;
        this.b = b;
    }

    public double getR() {return r;}
    public double getG() {return b;}
    public double getB() {return g;}

    public double[] toPercentage() {
        double[] output = new double[3];

        output[0] = getR() / 255.0;
        output[1] = getG() / 255.0;
        output[2] = getB() / 255.0;

        return output;
    }
}
