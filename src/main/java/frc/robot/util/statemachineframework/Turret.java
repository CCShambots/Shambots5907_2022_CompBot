package frc.robot.util.statemachineframework;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import javax.management.InstanceNotFoundException;

import static frc.robot.util.statemachineframework.Turret.TurretState.*;

public class Turret extends StatedSubsystem<Turret.TurretState> {

    public Turret() {
        super(TurretState.class);

        addDetermination(Undetermined, Idle, new PrintCommand("Undetermined->Idle"));

        addTransition(Idle, Active, new PrintCommand("Idle->Active")); //Enable limelight
        addTransition(Active, Idle, new PrintCommand("Active->Idle")); //Disable limelight
        addTransition(Idle, Odometry, new PrintCommand("Idle->Odometry"));
        addTransition(Odometry, Idle, new PrintCommand("Odometry->Idle"));
        addTransition(Odometry, Active, new PrintCommand("Odometry->Active")); //Enable limelight
        addTransition(Active, Odometry, new PrintCommand("Active->Odometry")); //Disable limelight

        setContinuousCommand(Active, new PrintCommand("Started Active continuous command")); //Actively track with limelight
        setContinuousCommand(Odometry, new PrintCommand("Started Odometry continuous command")); //Enable odometry

        addFlagState(Active, LockedIn, () -> true); //Indicate if the turret is locked in

    }

    public enum TurretState {
        Undetermined, Idle, Odometry, Active, LockedIn
    }

    @Override
    public void update() {

    }

    @Override
    public String getName() {
        return "ExampleTurret";
    }
}


