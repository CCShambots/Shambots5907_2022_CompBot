package frc.robot.util.statemachineframework;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.ArrayList;
import java.util.List;

public class Transition<E extends Enum<E>> {
    private E startState;
    private E endState;
    private E interruptionState;
    private Command command;

    public Transition(E startState, E endState, E interruptionState, Command command) {
        this.startState = startState;
        this.endState = endState;
        this.interruptionState = interruptionState;
        this.command = command;
    }

    public Transition(E startState, E endState, Command command) {
        this.startState = startState;
        this.endState = endState;
        this.interruptionState = startState;
        this.command = command;
    }

    public boolean isValidTransition(Transition testTransition) {
        if(testTransition.startState == this.startState && testTransition.endState == this.endState) return false;
        if(testTransition.command == this.command) return false;

        return true;
    }

    @Override
    public String toString() {
        return "Start state: " + this.startState.name() + ", Ending state: " + this.endState.name() + ", Command: " + this.command.toString();
    }

    public E getStartState() {return startState;}
    public E getEndState() {return endState;}
    public E getInterruptionState() {return interruptionState;}
    public Command getCommand() {return command;}
}
