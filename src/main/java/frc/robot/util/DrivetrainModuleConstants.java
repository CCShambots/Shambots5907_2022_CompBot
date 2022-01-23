package frc.robot.util;

public class DrivetrainModuleConstants{
    private double p;
    private double i;
    private double d;
    private double ks;
    private double kv;

    public DrivetrainModuleConstants(double p, double i, double d, double ks, double kv) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.ks = ks;
        this.kv = kv;
    }

    public double getP() {return p;}
    public double getI() {return i;}
    public double getD() {return d;}
    public double getKS() {return ks;}
    public double getKV() {return kv;}
}
