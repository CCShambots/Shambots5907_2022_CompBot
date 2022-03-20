package frc.robot.util.intake;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Conveyor.Direction;
import frc.robot.util.hardware.ColorSensor;
import frc.robot.util.hardware.ProximitySensor;
import frc.robot.util.intake.Ball.BallPosition;
import frc.robot.util.intake.Ball.BallColor;

public class BallTracker implements Sendable{

    private ProximitySensor stage1Sensor, stage2Sensor, stage3Sensor;
    private ColorSensor colorSensor;
    private Conveyor conveyor;

    private boolean prevStage1, prevStage2, prevStage3;

    private boolean disabled = false;
    private boolean error = false;

    private Ball emptyBall = new Ball(BallColor.NotInBot, BallPosition.NotInBot);

    //The balls that can exist that are currently in the robot
    List<Ball> balls = new ArrayList<>();

    public BallTracker(ProximitySensor stage1, ProximitySensor stage2, ProximitySensor stage3, ColorSensor colorSensor, Conveyor conveyor) {
        stage1Sensor = stage1;
        stage2Sensor = stage2;
        stage3Sensor = stage3;
        this.colorSensor = colorSensor;
        this.conveyor = conveyor;
    }

    public int getNumberOfBalls() {return balls.size();}

    //Both of these methods are protected from being null; they will return NotInBot if there is no ball at the expected index
    public BallPosition getBall1Pos() {return balls.size() >= 1 ? balls.get(0).getPosition() : BallPosition.NotInBot;}
    public BallPosition getBall2Pos() {return balls.size() >= 2 ? balls.get(1).getPosition() : BallPosition.NotInBot;}



    public void setCurrentState(List<Ball> balls) {
        this.balls = balls;
        this.error = false;
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
        boolean currStage3 = stage3Sensor.isActivated();

        //Only run the loop if the intake is moving
        if(conveyor.isRunning() && !disabled) {
            if(conveyor.getDirection() == Direction.Intake) {
                //Create a new ball in the conveyor if the stage 1 sensor has just turned on
                if(currStage1 && currStage1 != prevStage1) {
                    BallColor color = colorSensor.getColor() == Constants.allianceColor ? BallColor.Ours : BallColor.Opposing;
                    balls.add(new Ball(color));
                }

                //Advance the ball if the stage 1 sensor just got deactivated
                if(!currStage1 && currStage1 != prevStage1) {
                    safeAdvanceBall(BallPosition.Between1And2);
                }

                //If the second stage sensor was just activated, advance that ball to being in the second stage
                if(currStage2 && currStage2 != prevStage2) {
                    safeAdvanceBall(BallPosition.Stage2);
                }

                //If the second stage sensor was just deactivated, move it to past between stage 2 and 3
                if(!currStage2 && currStage2 != prevStage2) {
                    safeAdvanceBall(BallPosition.Between2And3);
                }

                //If the third stage sensor just activated, move it to stage 3
                if(currStage3 && currStage3 != prevStage3) {
                    safeAdvanceBall(BallPosition.Stage3);
                }

                //If the third sensor just turned off, move it out of the bot
                if(!currStage3 && currStage3 != prevStage3) {
                    safeAdvanceBall(BallPosition.NotInBot);
                }

            } else if(conveyor.getDirection() == Direction.Exhaust) {

                //If the third stage sensor deactivates, regress the ball to between 2 and 3
                if(!currStage3 && currStage3 != prevStage3) {
                    safeRegressBall(BallPosition.Between2And3);
                }

                //If the second stage sensor activates, regress the ball to Stage 2
                if(currStage2 && currStage2 != prevStage2) {
                    safeRegressBall(BallPosition.Stage2);
                }


                //If the second stage sensor deactivated, regress the ball once
                if(!currStage2 && currStage2 != prevStage2) {
                    safeRegressBall(BallPosition.Between1And2);
                }

                //If the first stage sensor activates, regress, the first or second ball's position (depending on how many are in the bot)
                if(currStage1 && currStage1 != prevStage1) {
                    safeRegressBall(BallPosition.Stage1);
                }
                                
                //If the first stage sensor deactivates, remove the ball (it's been ejected)
                if(!currStage1 && currStage1 != prevStage1) {
                    safeRegressBall(BallPosition.NotInBot);
                }   
                
            }
        }

        prevStage1 = currStage1;
        prevStage2 = currStage2;
        prevStage3 = currStage3;

        for(int i = 0; i<2; i++) {
            if(balls.size() > i) {
                SmartDashboard.putData("Ball #" + i, balls.get(i));
            } else {
                SmartDashboard.putData("Ball #" + i, emptyBall);
            }
        }

    }

    private void safeRegressBall(BallPosition pos) {
        //Find whichever ball is in the next position (i.e. where the ball is coming from)
        System.out.println("Searching for ball at position " + pos.next().name());
        Ball ball = findBall(pos.next());
        if(ball == null || !ball.isPrevPosition(pos)) {
            setError(true);
            return;
        }
        ball.regressPosition();

        if(pos == BallPosition.NotInBot) balls.remove(ball);
    }

    private void safeAdvanceBall(BallPosition pos) {
        System.out.println("Searching for ball at position " + pos.previous().name());
        Ball ball = findBall(pos.previous());
        if(ball == null || !ball.isNextPosition(pos)) {
            setError(true);
            return;
        }
        ball.setPosition(pos);

        //If the ball is no longer in the bot, we want to remove it from the list
        if(pos == BallPosition.NotInBot) balls.remove(ball);
    }

    private Ball findBall(BallPosition pos) {

        boolean failure = false;

        if(balls.size() >= 1) {
            if(balls.get(0).getPosition().equals(pos)) {
                System.out.println("Returning ball at index 0");
                return balls.get(0);
            } else if(balls.size() >= 2) {
                if(balls.get(1).getPosition().equals(pos)){
                    System.out.println("Returning ball index 1");
                    return balls.get(1);
                } else failure = true;
            }else failure = true;
        }

        if(failure) {
            System.out.println("Tracker error");
            setError(true);
            return new Ball(BallColor.Ours);
        } else return null; //This should never get called but makes the compiler happy
    }

    public void resetForAuto() {
        error = false;
        balls.clear();
        balls.add(new Ball(BallColor.Ours, BallPosition.Between2And3));
    }
    
    private void setError(boolean value) {error = value;}
    public boolean getError() {return error;}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Ball Tracker");

        builder.addBooleanProperty("Stage 1 sensor", () -> stage1Sensor.isActivated(), null);
        builder.addBooleanProperty("Stage 2 sensor", () -> stage2Sensor.isActivated(), null);
        builder.addBooleanProperty("Stage 3 sensor", () -> stage3Sensor.isActivated(), null);
    }

}
