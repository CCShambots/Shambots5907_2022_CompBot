package frc.robot.util.intake;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.util.hardware.PresenceSensor;
import frc.robot.util.intake.Ball.BallPosition;
import frc.robot.util.intake.Ball.Color;

public class BallTracker implements Sendable{
    //TODO: Color sensor implementation

    private PresenceSensor stage1Sensor;
    private PresenceSensor stage2Sensor;
    private Conveyor conveyor;

    private boolean prevStage1;
    private boolean prevStage2;

    //The balls that can exist that are currently in the robot
    List<Ball> balls = new ArrayList<>();

    public BallTracker(PresenceSensor stage1, PresenceSensor stage2, Conveyor conveyor) {
        stage1Sensor = stage1;
        stage2Sensor = stage2;
        this.conveyor = conveyor;
    }

    public int getNumberOfBalls() {return balls.size();}

    //Both of these methods are protected from being null; they will return NotInBot if there is no ball at the expected index
    public BallPosition getBall1Pos() {return balls.size() >= 1 ? balls.get(0).getPosition() : BallPosition.NotInBot;}
    public BallPosition getBall2Pos() {return balls.size() >= 2 ? balls.get(1).getPosition() : BallPosition.NotInBot;}

    public void setCurrentState(List<Ball> balls) {
        this.balls = balls;


    }

    public void periodic() {
        SmartDashboard.putNumber("Number of balls tracking", balls.size());

        boolean currStage1 = stage1Sensor.isActivated();
        boolean currStage2 = stage2Sensor.isActivated();

        //Only run the loop if the intake is moving
        //TODO: Remove this always true once things are better defined
        if(conveyor.isRunning() || true) {
            //Create a new ball in the conveyor if the stage 1 sensor has just turned on
            if(currStage1 && currStage1 != prevStage1) {
                balls.add(new Ball(Color.Ours));
            }

            //Advance the ball if the stage 1 sensor just got deactivated
            if(!currStage1 && currStage1 != prevStage1) {
                balls.get(balls.size()-1).advancePosition();
            }

            //If the second stage sensor was just activated, advance that ball to being in the second stage
            if(currStage2 && currStage2 != prevStage2) {
                balls.get(0).advancePosition();
            }

            //If the second stage sensor was just deactivated, remove that ball from the list (it exited the bot)
            if(!currStage2 && currStage2 != prevStage2) {
                balls.remove(0);
            }
        }

        prevStage1 = currStage1;
        prevStage2 = currStage2;

        for(int i = 0; i<2; i++) {
            if(balls.size() > i) {
                SmartDashboard.putData("Ball #" + i, balls.get(i));
            } else {
                SmartDashboard.putData("Ball #" + i, new Ball(Color.Ours, BallPosition.NotInBot));
            }
        }

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Smart Dashboard");

        builder.addBooleanProperty("Stage one sensor", () -> stage1Sensor.isActivated(), null);
        builder.addBooleanProperty("Stage two sensor", () -> stage2Sensor.isActivated(), null);
    }
}
