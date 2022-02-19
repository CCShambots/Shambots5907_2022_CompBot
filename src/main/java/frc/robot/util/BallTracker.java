package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Ball.BallPosition;
import frc.robot.util.Ball.Color;
import frc.robot.util.sensors.PresenceSensor;

public class BallTracker{
    //TODO: Color sensor implementation

    private PresenceSensor stage1Sensor;
    private PresenceSensor stage2Sensor;

    private boolean prevStage1;
    private boolean prevStage2;

    //The balls that can exist that are currently in the robot
    List<Ball> balls = new ArrayList<>();

    public BallTracker(PresenceSensor stage1, PresenceSensor stage2) {
        stage1Sensor = stage1;
        stage2Sensor = stage2;
    }

    public int getNumberOfBalls() {return balls.size();}

    //Both of these methods are protected from being null; they will return NotInBot if there is no ball at the expected index
    public BallPosition getBall1Pos() {return balls.get(0) != null ? balls.get(0).getPosition() : BallPosition.NotInBot;}
    public BallPosition getBall2Pos() {return balls.get(1) != null ? balls.get(1).getPosition() : BallPosition.NotInBot;}

    public void update() {
        SmartDashboard.putNumber("Number of balls tracking", balls.size());
        for(Ball b : balls) {
            SmartDashboard.putData(b);
        }

        //TODO: Remove this telemetry once we know how things are going to work
        SmartDashboard.putBoolean("Stage one sensor", stage1Sensor.isActivated());
        SmartDashboard.putBoolean("Stage two sensor", stage2Sensor.isActivated());

        //Create a new ball in the conveyor if the stage 1 sensor has just turned on
        if(stage1Sensor.isActivated() && stage1Sensor.isActivated() != prevStage1) {
            balls.add(new Ball(Color.Ours));
        }

        //Advance the ball if the stage 1 sensor just got deactivated
        if(!stage1Sensor.isActivated() && stage1Sensor.isActivated() != prevStage1) {
            balls.get(balls.size()-1).advancePosition();
        }

        //If the second stage sensor was just activated, advance that ball to being in the second stage
        if(stage2Sensor.isActivated() && stage2Sensor.isActivated() != prevStage2) {
            balls.get(0).advancePosition();
        }

        //If the second stage sensor was just deactivated, remove that ball from the list (it exited the bot)
        if(!stage2Sensor.isActivated() && stage2Sensor.isActivated() != prevStage2) {
            balls.remove(0);
        }

        prevStage1 = stage1Sensor.isActivated();
        prevStage2 = stage2Sensor.isActivated();
    }
}
