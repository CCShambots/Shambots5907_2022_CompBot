package frc.robot.util.intake;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Ball implements Sendable{
    private BallColor color;
    private BallPosition position;

    public Ball(BallColor color) {
        this.color = color;
        position = BallPosition.Stage1;
    }

    public Ball(BallColor color, BallPosition position) {
        this.color = color;
        this.position = position;
    }

    void advancePosition() {position = position.next();}
    void regressPosition() {position = position.previous();}
    void setPosition(BallPosition pos) {position = pos;}
    BallPosition getPosition() {return position;}
    BallColor getColor() {return color;}
    
    /**
     * @param pos The position that we're trying to move the ball to
     * @return whether that is indeed the next position that should next be moved to
     */
    boolean isNextPosition(BallPosition pos) {return position.next().equals(pos);}

    /**
     * @param pos The position that we're trying to move the ball to
     * @return whether that is indeed the previous position that the ball should go to
     */
    boolean isPrevPosition(BallPosition pos) {return position.previous().equals(pos);}

    public static enum BallPosition {
        Stage1, //The first sensor IS activated
        Between1And2, //The first sensor WAS activated, but the second has not yet been
        Stage2, //The second presence sensor IS activated
        Between2And3, //The second presence sensor WAS activated
        Stage3, //The third presence sensor IS activated
        NotInBot; //The ball has left the robot

        private static BallPosition[] vals = values();
        public BallPosition next() { return vals[(this.ordinal() +1) % vals.length];}
        public BallPosition previous() { 
            if(this.ordinal() - 1 == -1) return BallPosition.NotInBot;
            return vals[(this.ordinal() -1) % vals.length];
        }

    }

    public static enum BallColor {
        Ours, //Our alliance's colored balls
        Opposing, //The opposing alliance's colored balls
        NotInBot, //The ball is not in the robot 
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Ball");
        builder.addStringProperty("Position", () -> getPosition().name(), null);
        builder.addStringProperty("Color", () -> getColor().name(), null);
        
    }
}
