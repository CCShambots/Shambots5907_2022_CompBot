package frc.robot.util;

/**
 * Container class for storing information for feedforward and Pid control (supports profiled PID controller values, which are set in an alternate constructor)
 */
public class PIDandFFConstants{
    private double p;
    private double i;
    private double d;
    private double ks;
    private double kv;
    private double maxAccel;
    private double maxVel;

    public PIDandFFConstants(double p, double i, double d, double ks, double kv) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.ks = ks;
        this.kv = kv;
        this.maxAccel = 0;
        this.maxVel = 0;
    }

    public PIDandFFConstants(double p, double i, double d, double ks, double kv, double maxVel, double maxAccel) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.ks = ks;
        this.kv = kv;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
    }

    public double getP() {return p;}
    public double getI() {return i;}
    public double getD() {return d;}
    public double getKS() {return ks;}
    public double getKV() {return kv;}
    public double getMaxVel() {return maxVel;}
    public double getMaxAccel() {return maxAccel;}
}
