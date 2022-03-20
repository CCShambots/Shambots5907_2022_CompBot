package frc.robot.util.lights;

import java.awt.Color;

public class HSV {
    private float h;
    private float s;
    private float v;

    public HSV(float h, float s, float v) {
        this.h = h;
        this.s = s;
        this.v = v;
    }

    public float getH() {return h;}
    public float getS() {return s;}
    public float getV() {return v;}

    public static HSV fromFloatArray(float[] arr) {
        return new HSV(arr[0], arr[1], arr[2]);
    }

    public RGB toRGB() {
        int rgb = Color.HSBtoRGB(getH(), getS(), getV());
        int red = (rgb >> 16) & 0xFF;
        int green = (rgb >> 8) & 0xFF;
        int blue = rgb & 0xFF;

        return new RGB(red, green, blue);
    }
}
