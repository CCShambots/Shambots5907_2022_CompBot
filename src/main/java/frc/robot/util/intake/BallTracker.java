package frc.robot.util.intake;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Conveyor.Direction;
import frc.robot.util.hardware.ProximitySensor;
import frc.robot.util.intake.Ball.BallPosition;
import frc.robot.util.intake.Ball.Color;

public class BallTracker implements Sendable{
    //TODO: Color sensor implementation
    //TODO: Support for ejecting balls

    private ProximitySensor stage1Sensor;
    private ProximitySensor stage2Sensor;
    private Conveyor conveyor;

    private boolean prevStage1;
    private boolean prevStage2;

    private boolean disabled = false;

    private Ball emptyBall = new Ball(Color.Ours, BallPosition.NotInBot);

    //The balls that can exist that are currently in the robot
    List<Ball> balls = new ArrayList<>();

    public BallTracker(ProximitySensor stage1, ProximitySensor stage2, Conveyor conveyor) {
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

    public void setDisabled(boolean value) { 
        disabled = value;
    }

    public void setPrevSensorStates(boolean stage1, boolean stage2) {
        prevStage1 = stage1;
        prevStage2 = stage2;
    }

    public void periodic() {
        SmartDashboard.putNumber("Number of balls tracking", balls.size());

        boolean currStage1 = stage1Sensor.isActivated();
        boolean currStage2 = stage2Sensor.isActivated();

        //Only run the loop if the intake is moving
        //TODO: Remove always true later
        if(conveyor.isRunning() && !disabled) {
            if(conveyor.getDirection() == Direction.Intake) {
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
                    balls.get(0).advancePosition();
                }
            } else if(conveyor.getDirection() == Direction.Exhaust) {
                //If the second stage sensor deactivated, regress the ball once
                if(!currStage2 && currStage2 != prevStage2) {
                    balls.get(0).regressPosition();
                }
                
                //If the first stage sensor deactivates, remove the ball (it's been ejected)
                if(!currStage1 && currStage1 != prevStage1) {
                    balls.remove(balls.size()-1);
                }   
                
                //If the first stage sensor activates, regress, the first or second ball's position (depending on how many are in the bot)
                if(currStage1 && currStage1 != prevStage1) {
                    if(balls.get(0).getPosition() == BallPosition.BetweenStages) balls.get(0).regressPosition();
                    else if(balls.size() > 1) balls.get(1).regressPosition();
                }
            }
        }

        prevStage1 = currStage1;
        prevStage2 = currStage2;

        for(int i = 0; i<2; i++) {
            if(balls.size() > i) {
                SmartDashboard.putData("Ball #" + i, balls.get(i));
            } else {
                SmartDashboard.putData("Ball #" + i, emptyBall);
            }
        }

    }

    public void resetForAuto() {
        balls.clear();
        balls.add(new Ball(Color.Ours, BallPosition.PastStage2));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Ball Tracker");

        builder.addBooleanProperty("Stage one sensor", () -> stage1Sensor.isActivated(), null);
        builder.addBooleanProperty("Stage two sensor", () -> stage2Sensor.isActivated(), null);
    }
}
