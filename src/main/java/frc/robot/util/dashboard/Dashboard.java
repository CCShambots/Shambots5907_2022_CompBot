package frc.robot.util.dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Robot;

public class Dashboard {

    private Robot robot;
    private NetworkTable table;

    public Dashboard(Robot robot) {
        this.robot = robot;
        this.table = NetworkTableInstance.getDefault().getTable("dashboard");

        setTab(Tab.Auto);

    }


    public void setTab(Tab t) {
        table.getEntry("tab").setString(t.name());
    }

    public enum Tab {
        Auto, Teleop, Test, Debug, Pathing
    }


}
