package frc.robot.commands.intake;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.util.intake.Ball;
import frc.robot.util.intake.Ball.BallColor;
import frc.robot.util.intake.Ball.BallPosition;
import static frc.robot.Constants.Turret.*;
import static frc.robot.Constants.Conveyor.*;

public class IntakeCommand extends CommandBase{

    //Config values
    private double startIntakeDelay = .5; //The time (in seconds) the intake should delay before starting
    
    private Intake intake;
    private Conveyor conveyor;
    private Turret turret;
    private Drivetrain drivetrain;

    private State state = State.Normal;
    private boolean finished = false;
    private Timer startDelayTimer;

    private boolean ejectingOutTop;
    private Ball ejectingBall;

    private boolean ejectingOutBottom;
    private Timer ejectionTimer;
    
    /**
     * Creates a new intake command and starts running it.
     * CONSTRAINTS: REQUIRES that the robot not already have two balls in it (one is fine and will be handled in initialize())
     * @param intake
     * @param conveyor
     * @param cancelSupplier
     */
    public IntakeCommand(Intake intake, Conveyor conveyor, Turret turret, Drivetrain drivetrain) {
        this.intake = intake;
        this.conveyor = conveyor;
        this.turret = turret;
        this.drivetrain = drivetrain;

        ejectionTimer = new Timer();
        startDelayTimer = new Timer();

        addRequirements(intake, conveyor);
    }

    @Override
    public void initialize() {
        state = State.Normal;
        finished = false;
        ejectingOutTop = false;
        ejectingBall = null;
        ejectingOutBottom = false;

        intake.setShouldEnd(false);
        intake.lowerIntake();
        conveyor.intakeStage1();

        if(conveyor.getNumberOfBalls() == 0) {conveyor.intakeStage2();}
        startDelayTimer.stop();
        startDelayTimer.reset();
        startDelayTimer.start();

        ejectionTimer.stop();
        ejectionTimer.reset();
    }

    @Override
    public void execute() {

        //Runs once after the intake has moved far enough down
        if(startDelayTimer.get() > startIntakeDelay)  {
            startDelayTimer.stop();
            startDelayTimer.reset();
            intake.intake(); 
        }

        if(drivetrain.isDefending()) {
            defendingLoop();
        } else {
            normalLoop();
        }

        //If stopping is indicated by the flag in the intake subsystem, the robot will immediately stop and raise the intake.
        //The command will finish as soon as balls are correctly processed
        if(state == State.Normal && intake.getShouldEnd()) {
            intake.raiseIntake();
            state = State.Cancelling;
        }
    }

    private void normalLoop() {
        if(state == State.Normal) {
            if(conveyor.getBall1Color() == BallColor.Opposing && !ejectingOutTop) {
                ejectingOutTop = true;
                ejectingBall = conveyor.getBall1();
                turret.setFlywheelTarget(FLYWHEEL_LOW_RPM);
            }

            //If the first ball (i.e. the one that entered the robot first) has reached Stage 2, we will stop the stage 2 conveyor
            if(conveyor.getBall1Pos() == BallPosition.Between2And3 && !ejectingOutTop) {conveyor.stopStage2();}

            if(conveyor.getBall2Pos() == BallPosition.Stage1) {
                intake.stop();
                intake.raiseIntake();
                if(conveyor.getBall2Color() == BallColor.Ours) {
                    conveyor.stopStage1();
                    finished = true;
                } else {
                    ejectingOutBottom = true;
                    conveyor.exhaustStage1();
                    intake.stop();
                    intake.raiseIntake();
                }
            } else if(ejectingOutBottom && conveyor.getBall2Pos() == BallPosition.NotInBot){
                ejectionTimer.start();
            }

            if(ejectionTimer.get() > EJECTION_DELAY) {
                conveyor.stopAll();
                intake.setShouldEnd(true);
            }

        } else if(state == State.Cancelling) {
            if(!ejectingOutTop) {
                if( !(conveyor.getBall1Pos() == BallPosition.Between1And2 || conveyor.getBall1Pos() == BallPosition.Stage2 || conveyor.getBall1Pos() == BallPosition.Stage1)) {
                    conveyor.stopAll();
                    finished = true;
                }
            }
        }

        if(!conveyor.getBall1().equals(ejectingBall) && ejectingOutTop) {
            ejectingOutTop = false;
            ejectingBall = null;
            turret.setFlywheelTarget(0);
        }
    }

    private void defendingLoop() {
        

        if(state == State.Normal) {
            //If the first ball (i.e. the one that entered the robot first) has reached Stage 2, we will stop the stage 2 conveyor
            if(conveyor.getBall1Pos() == BallPosition.Between2And3) {conveyor.stopStage2();}

            //If the second ball has reached stage 1, we will end the command and stop and raise the intake/conveyor
            if(conveyor.getBall2Pos() == BallPosition.Stage1) {
                conveyor.stopStage1();
                finished = true;
            }
        } else if(state == State.Cancelling) {
            //If the frist ball is not between two stages, we can safely end the command
            if(!(conveyor.getBall1Pos() == BallPosition.Between1And2 || conveyor.getBall1Pos() == BallPosition.Stage2 || conveyor.getBall1Pos() == BallPosition.Stage1)) {
                conveyor.stopAll();
                finished = true;
            }
            
            intake.stop();
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
            conveyor.stopAll();
        }

        intake.raiseIntake();
        intake.stop();
        
        startDelayTimer.stop();
        ejectionTimer.stop();
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
