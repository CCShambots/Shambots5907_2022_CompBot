package frc.robot.util.statemachineframework;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import static frc.robot.util.statemachineframework.ExampleSubsystem.ExampleState.*;

public class ExampleSubsystem extends StatedSubsystem<ExampleSubsystem.ExampleState> {

    public ExampleSubsystem() {
        super(ExampleState.class);

        addDetermination(Undetermined, Idle, new InstantCommand());
        addTransition(Idle, Running, new InstantCommand());
        addTransition(Running, Idle, new InstantCommand());

        setContinuousCommand(Running, new InstantCommand());

    }

    public enum ExampleState {
        Undetermined, Idle, Running
    }

    @Override
    public void update() {

    }
}


