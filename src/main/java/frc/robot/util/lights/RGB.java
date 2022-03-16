package frc.robot.util.lights;

import java.awt.Color;

public class RGB {
    private int r;
    private int g;
    private int b;

    /**
     * 
     * @param r red channel (0-255)
     * @param g green channel (0-255)
     * @param b blue channel (0-255)
     */
    public RGB(int r, int g, int b) {
        this.r = r;
        this.g = g;
        this.b = b;
    }

    public int getR() {return r;}
    public int getG() {return g;}
    public int getB() {return b;}

    public double[] toPercentage() {
        double[] output = new double[3];

        output[0] = getR() / 255.0;
        output[1] = getG() / 255.0;
        output[2] = getB() / 255.0;

        return output;
    }

    public HSV toHSV() {
        return HSV.fromFloatArray(Color.RGBtoHSB(getR(), getG(), getB(), null));
    }
}
