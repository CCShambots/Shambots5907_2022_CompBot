package frc.robot.commands.turret.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Turret;
import frc.robot.util.intake.Ball.BallPosition;

public class ShootCommand extends CommandBase {
    
    private Conveyor conveyor;
    private Turret turret;
    
    private boolean finished = false;
    private State state;

    public ShootCommand(Conveyor conveyor, Turret turret) {
        this.conveyor = conveyor;
        this.turret = turret;
    }

    @Override
    public void initialize() {
        conveyor.intakeStage2();

        finished = false;
        
        turret.setIsShooting(true);

        if(conveyor.getNumberOfBalls() > 1) conveyor.intakeStage1();
        
        state = State.Ejecting;
    }

    @Override
    public void execute() {

        if(conveyor.getNumberOfBalls() == 0) {
            finished = true;
            return;
        }
        
        if(state == State.Ejecting) {
            if(conveyor.getBall1Pos() == BallPosition.Stage3 && turret.isFlywheelBusy()) {
                conveyor.stopStage2();
                state = State.Holding;
            }
        } else if(state == State.Holding) {
            if(!turret.isFlywheelBusy()) {
                conveyor.intakeStage2();
                state = State.Ejecting;
            }
        }

        
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stopAll();
        conveyor.clearTracker();

        turret.setIsShooting(false);
    }

    public static enum State {Ejecting, Holding}
}
