package frc.robot.util.intake;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Ball implements Sendable{
    private Color color;
    private BallPosition position;

    public Ball(Color color) {
        this.color = color;
        position = BallPosition.Stage1;
    }

    public Ball(Color color, BallPosition position) {
        this.color = color;
        this.position = position;
    }

    public void advancePosition() {position = position.next();}
    public void regressPosition() {position = position.previous();}
    public void setPosition(BallPosition pos) {position = pos;}
    public BallPosition getPosition() {return position;}
    public Color getColor() {return color;}

    public static enum BallPosition {
        Stage1, //The first sensor IS activated
        BetweenStages, //The first sensor WAS activated, but the second has not yet been
        Stage2, //The second presence sensor IS activated
        NotInBot; //The ball has left the robot

        private static BallPosition[] vals = values();
        public BallPosition next() { return vals[(this.ordinal() +1) % vals.length];}
        public BallPosition previous() { return vals[(this.ordinal() -1) % vals.length];}

    }

    public static enum Color {
        Ours, //Our alliance's colored balls
        Opposing //The opposing alliance's colored balls
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Ball");
        builder.addStringProperty("Position", () -> getPosition().name(), null);
        builder.addStringProperty("Color", () -> getColor().name(), null);
        
    }
}
