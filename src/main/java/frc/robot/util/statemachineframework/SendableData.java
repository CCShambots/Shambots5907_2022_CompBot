package frc.robot.util.statemachineframework;

public class SendableData {
    private Type type;

    private boolean boolVal;
    private boolean[] boolArrVal;
    private double doubleVal;
    private double[] doubleArrVal;
    private String stringVal;
    private String[] stringArrVal;

    public SendableData(boolean boolVal) {
        this.boolVal = boolVal;
        this.type = Type.Boolean;
    }

    public SendableData(boolean[] boolArrVal) {
        this.boolArrVal = boolArrVal;
        this.type = Type.BooleanArray;
    }

    public SendableData(double doubleVal) {
        this.doubleVal = doubleVal;
        this.type = Type.Double;
    }

    public SendableData(double[] doubleArrVal) {
        this.doubleArrVal = doubleArrVal;
        this.type = Type.DoubleArray;
    }

    public SendableData(String stringVal) {
        this.stringVal = stringVal;
        this.type = Type.String;
    }

    public SendableData(String[] stringArrVal) {
        this.stringArrVal = stringArrVal;
        this.type = Type.StringArray;
    }


    public enum Type {
        Boolean, Double, String, BooleanArray, DoubleArray, StringArray
    }
}
