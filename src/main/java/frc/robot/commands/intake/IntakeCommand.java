package frc.robot.commands.intake;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.util.intake.Ball.BallPosition;
import frc.robot.util.lights.RGB;
import frc.robot.util.lights.animations.BlinkingAnimation;
import frc.robot.util.lights.animations.SolidAnimation;

public class IntakeCommand extends CommandBase{
    
    private Intake intake;
    private Conveyor conveyor;

    private State state = State.Normal;
    private boolean finished = false;
    private Timer timer;

    private double startIntakeDelay = .5; //The time (in seconds) the intake should delay before starting
    
    /**
     * Creates a new intake command and starts running it.
     * CONSTRAINTS: REQUIRES that the robot not already have two balls in it (one is fine and will be handled in initialize())
     * @param intake
     * @param conveyor
     * @param cancelSupplier
     */
    public IntakeCommand(Intake intake, Conveyor conveyor) {
        this.intake = intake;
        this.conveyor = conveyor;


        addRequirements(intake, conveyor);
    }

    @Override
    public void initialize() {
        state = State.Normal;
        finished = false;

        intake.setShouldEnd(false);
        intake.lowerIntake();
        conveyor.intakeStage1();

        if(conveyor.getNumberOfBalls() == 0) {conveyor.intakeStage2();}
        timer = new Timer();
        timer.start();

    }

    @Override
    public void execute() {
        if(timer.get() > startIntakeDelay)  {
            timer.stop();
            intake.intake(); 
        }

        if(state == State.Normal) {
            //If the first ball (i.e. the one that entered the robot first) has reached Stage 2, we will stop the stage 2 conveyor
            if(conveyor.getBall1Pos() == BallPosition.PastStage2) {conveyor.stopStage2();}

            //If the second ball has reached stage 1, we will end the command and stop and raise the intake/conveyor
            if(conveyor.getBall2Pos() == BallPosition.Stage1) {
                conveyor.stopStage1();
                intake.stop();
                intake.raiseIntake();
                finished = true;
            }
        } else if(state == State.Cancelling) {
            //If the frist ball is not between two stages, we can safely end the command
            if(!(conveyor.getBall1Pos() == BallPosition.BetweenStages || conveyor.getBall1Pos() == BallPosition.Stage2) && !(conveyor.getBall1Pos() == BallPosition.Stage1)) {
                conveyor.stopAll();
                finished = true;
            }
            
            intake.stop();
        }

        //If stopping is indicated by the drive team, the robot will immediately stop and raise the intake.
        //The command will finish once there is no ball between stages
        if(state == State.Normal && intake.getShouldEnd()) {
            intake.raiseIntake();
            state = State.Cancelling;
        }

    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    /**
     * We will stop the intake if the command is interrupted (which it never should be unless the robot is disabled)
     */
    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            intake.stop();
            intake.raiseIntake();
            conveyor.stopAll();
        }
        intake.stop();
        
        timer.stop();
    }

    private static enum State {
        Normal, Cancelling
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // super.initSendable(builder);
        builder.setSmartDashboardType("Intake Command");
        builder.addStringProperty("Current status", () -> state.name(), null);
    }
    
}
