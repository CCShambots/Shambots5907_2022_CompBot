package frc.robot.util.statemachineframework;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.*;
import java.util.function.BooleanSupplier;
import java.util.stream.Collectors;

public abstract class StatedSubsystem<E extends Enum<E>> extends SubsystemBase {

    private final List<Transition<E>> transitions = new ArrayList<>();
    private final Map<E, List<FlagState<E>>> flagStates;
    private final Map<E, Command> continualCommands;
    private E undeterminedState;
    private E entryState;

    private E currentState;
    private E flagState;
    private E desiredState;

    private Transition<E> currentTransition;
    private boolean transitioning = false;
    private Command currentCommand = null;
    private boolean needToScheduleTransitionCommand = false;
    private boolean enabled = false;

    public StatedSubsystem(Class<E> enumType) {
        flagStates = new EnumMap<>(enumType);
        continualCommands = new EnumMap<>(enumType);
    }

    /**
     * Default version of this method reverts to the start state in case of an interruption
     * @param startState where the subsystem is when beginning the transition
     * @param endState where the subsystem ends the transition
     * @param command the command that runs
     */
    protected void addTransition(E  startState, E endState, Command command) {
        addTransition(startState, endState, startState, command);
    }

    /**
     * Alternate version of the method where an interruption state can be stated
     * @param startState where the subsystem is when beginning the transition
     * @param endState where the subsystem ends the transition
     * @param interruptionState the state that should be switched to if the transition is interrupted
     * @param command the command that runs through the transition
     */
    protected void addTransition(E startState, E endState, E interruptionState, Command command) {
        Transition<E> suggestedTransition = new Transition<>(startState, endState, interruptionState, command);

        //End the function if the transition conflicts
        if(!checkIfValidTransition(suggestedTransition)) return;

        transitions.add(suggestedTransition);
    }

    /**
     * Give the command that allows the
     * @param undeterminedState The state in which the subsystem is initialized
     * @param entryState The state the subsystem can go to from the undetermined state
     * @param command The command that runs in the transition
     */
    protected void addDetermination(E undeterminedState, E entryState, Command command) {
        if(this.undeterminedState != null) {
            outputErrorMessage("You've already defined an undetermined state for this subsystem",
                    "(Subsystem name: " + getName() + ")");
            return;
        }

        Transition<E> determinationTransition = new Transition<>(undeterminedState, entryState, command);

        if(!checkIfValidTransition(determinationTransition)) return;

        this.undeterminedState = undeterminedState;
        this.entryState = entryState;
        this.currentState = undeterminedState;
        this.desiredState = undeterminedState;

        transitions.add(determinationTransition);
    }

    /**
     * Add a new flag state to better indicate the subsystem's state
     * If a subsystem fulfills two flag states at once, the one it displays will be unreliable
     * NOTE: If you add two of the same flag state, unexpected behavior will occur
     */
    protected void addFlagState(E parentState, E flagState, BooleanSupplier condition) {
        if(!flagStates.containsKey(parentState)) flagStates.put(parentState, new ArrayList<>());

        if(getStatesMarkedAsFlag().contains(parentState)) {
            outputErrorMessage("FLAG STATES CANNOT BE USED AS PARENT STATES",
                    "You attempted to register the following:",
                    "Parent state: " + parentState.name(),
                    "Flag state: " + flagState.name());
        }

        flagStates.get(parentState).add(new FlagState<>(flagState, condition));
    }

    /**
     * Set a command to schedule once a state is reached.
     * This command should run indefinitely (i.e. isFinished() should never return true).
     * The command will be canceled when a transition begins.
     * Only the first command registered for a certain state will be
     */
    public void setContinuousCommand(E state, Command command) {
        if(!isValidCommand(command)) {
            outputErrorMessage("YOU TRIED TO DEFINE AN INVALID CONTINUOUS COMMAND",
                    "SUBSYSTEM NAME: " + getName(),
                    "STATE: " + state.name());
            return;
        }

        if(!continualCommands.containsKey(state)) continualCommands.put(state, command);
    }

    /**
     * Determined whether a given command is valid.
     * Commands must either require ONLY the subsystem for which they are being used, OR have no requirements
     * @param command the proposed command
     * @return if the proposed command is valid or not
     */
    private boolean isValidCommand(Command command) {
        //Commands should only require this one subsystem or no subsystem at all
        if(command.getRequirements().size() > 1) return false;
        else return command.getRequirements().contains(this) || command.getRequirements().size() != 1;
    }

    /**
     * @param proposedTransition The transition that should be compared against all the other transitions
     * @return whether the proposed transition is valid or not
     */
    private boolean checkIfValidTransition(Transition proposedTransition) {
        //The transition is invalid if it leads from or to the undetermined state
        if(this.undeterminedState == proposedTransition.getStartState() || this.undeterminedState == proposedTransition.getEndState()) return false;

        //The transition is invalid if the command itself is invalid (has incorrect subsystem requirements)
        if(!isValidCommand(proposedTransition.getCommand())) return false;

        //The transition is invalid if either one of its states have been marked as flag states
        Set<E> statesMarkedAsFlag = getStatesMarkedAsFlag();
        if(statesMarkedAsFlag.contains(proposedTransition.getStartState()) || statesMarkedAsFlag.contains(proposedTransition.getEndState()) || statesMarkedAsFlag.contains(proposedTransition.getInterruptionState())) {
            outputErrorMessage("TRANSITIONS CANNOT INCLUDE STATES MARKED AS TRANSITION STATES",
                    "TRANSITION TRYING TO BE ADDED: " + proposedTransition);
        }

        //Check the other defined transitions to see if there are any conflicts (if they represent the same states, or if they
        for(Transition<E> t : transitions) {
            if(!proposedTransition.isValidTransition(t)) {
                outputErrorMessage("YOU TRIED TO DEFINE AN INVALID TRANSITION",
                        "SUBSYSTEM NAME: " + getName(),
                        "EXISTING TRANSITION CONFLICT: " + t,
                        "TRANSITION TRYING TO BE ADDED: " + proposedTransition,
                        "NOTE: THIS TRANSITION WILL NOT WORK UNTIL THE ERROR IS CORRECTED");
                return false;
            }
        }

        return true;
    }

    @Override
    public final void periodic() {

        //States can only be managed whilst the subsystem is enabled
        if(enabled) {

            //If a command needs to be scheduled, and the previous command that was canceled is no longer scheduled, schedule the transition command
            if(needToScheduleTransitionCommand && !currentCommand.isScheduled()) {
                needToScheduleTransitionCommand = false;
                currentCommand = currentTransition.getCommand();
                currentCommand.schedule();
            }

            //If a transition has finished, do the following
            if(transitioning && !currentCommand.isScheduled()) {
                transitioning = false;
                currentState = desiredState;
                flagState = null;
                currentCommand = continualCommands.get(currentState);
                if(currentCommand != null) {
                    currentCommand.schedule();
                }
            }

            //Check for flag states and activate exactly one of them
            //There is only one flag position potentially active at a time
            //Also negate the existing flag state if its condition is no longer true

            for(FlagState<E> f : flagStates.get(currentState)) {
                if(f.getCondition().getAsBoolean()) {
                    flagState = f.getState();
                } else if(f.getState() == flagState) {
                    flagState = null;
                }
            }
        }

        update();
    }

    /**
     * Method that acts as a replacement for periodic()
     */
    public abstract void update();

    /**
     * Ask for the subsystem to move to a different state
     * If a flag state is provided, the robot will start transitioning to its parent state
     * @param state The state to which the subsystem should go
     */
    public void requestTransition(E state) {

        //Since this sate has been marked as a flag state, we find its parent state and request to transition to that
        if(getStatesMarkedAsFlag().contains(state)) {
            state = findParentState(state);
        }

        if(!getEndStates().contains(state)) {
            outputErrorMessage("YOU TRIED TO REQUEST A STATE THAT DOESN'T EXIST",
                    "SUBSYSTEM NAME: " + getName(),
                    "TRANSITION CONFLICT: " + state.name());
            return;
        }

        Transition<E> t = findTransition(currentState, state, transitions);

        //If a valid transition was found, then start performing it
        if(t != null) {

            //If a transition is already occurring, cancel it
            if(transitioning) {
                cancelTransition();
            }

            //If a continuous command is running, cancel it
            if(!transitioning && currentCommand != null) {
                currentCommand.cancel();
            }

            desiredState = state;
            currentTransition = t;
            needToScheduleTransitionCommand = true;
            transitioning = true;
        }
    }

    /**
     * Search the list of transitions to find which transition should be implemented
     * @return Whatever transition is found. This can be null
     */
    private Transition<E> findTransition(E startState, E endState, List<Transition<E>> transitions) {
        for(Transition<E> t : transitions) {
            if(startState==t.getStartState() && endState==t.getEndState()) return t;
        }

        return null;
    }

    public E findParentState(E state) {
        for(Map.Entry<E, List<FlagState<E>>> entry: flagStates.entrySet()) {
            if(entry.getValue().stream().map(FlagState::getState).collect(Collectors.toList()).contains(state)) return entry.getKey();
        }

        //Simply return null if it's not found (this function should only be run once it has been verified that the input state is a flag state
        return null;
    }

    public void requestDetermination() {
        requestTransition(entryState);
    }

    /**
     * Cancel any transition currently running
     */
    public void cancelTransition() {
        if(transitioning) {
            currentCommand.cancel();
            currentState = currentTransition.getInterruptionState();
        }
    }

    public void outputErrorMessage(String message, String... args) {
        System.out.println("-----" + message + "!!!!-----");
        for(String a : args) {
            System.out.println(a);
        }
        System.out.println("-------------------------------------------------------");
    }

    private List<E> getStartStates() {
        return transitions.stream().map(Transition::getStartState).collect(Collectors.toList());
    }

    private List<E> getEndStates() {
        return transitions.stream().map(Transition::getEndState).collect(Collectors.toList());
    }

    private List<Command> getCommands() {
        return transitions.stream().map(Transition::getCommand).collect(Collectors.toList());
    }

    /**
     * Get a set of all states that have been marked as flag states
     * @return all states marked as flags
     */
    private Set<E> getStatesMarkedAsFlag() {
        Set<E> states = new HashSet<>();

        for(List<FlagState<E>> flagState : flagStates.values()) {
            states.addAll(flagState.stream().map(FlagState::getState).collect(Collectors.toSet()));
        }

        return states;
    }

    /**
     * Determine whether the subsystem is actively running a command to transition between states
     * @return transitioning
     */
    public boolean isTransitioning() {return transitioning;}

    /**
     * @return Either the current state, or the current flag state (if there is a flag state)
     */
    public E getCurrentState() {return flagState != null ? flagState : currentState;}

    /**
     * @return the state the robot is trying to enter
     */
    public E getDesiredState() {return desiredState;}

    /**
     * @param state the state to check
     * @return Whether the subsystem is either in the given state or child flag state
     */
    public boolean isInState(E state) {return currentState == state || flagState == state;}

    public E getParentState() {return currentState;}
    public E getFlagState() {return flagState;}

    /**
     * @return A prettily formatted name for the subsytem (just the class name)
     */
    public String getName() {
        String fullName = this.getClass().getName();

        return fullName.substring(fullName.lastIndexOf(".") + 1);
    }

    /**
     * A command that waits indefinitely for a state to arrive and cancels whatever transition is ongoing if interrupted
     * @return Command from the factory
     */
    public Command waitForState(E state) {
        return new FunctionalCommand(() -> {}, () -> {}, (interrupted) -> {if(interrupted) cancelTransition();}, () -> isInState(state));
    }

    /**
     * A command that actively requests a state transition
     * @param state
     * @return
     */
    public Command goToState(E state) {
        return new FunctionalCommand(() -> {requestTransition(state);}, () -> {}, (interrupted) -> {if(interrupted) cancelTransition();}, () -> getCurrentState() == state);
    }

    /**
     * Tell the subsystem whether the subsystem is enabled or not
     * NOTE: Just because a SUBSYSTEM is enabled, that doesn't mean that the ROBOT is enabled
     * I.e. A subsystem for managing LEDs, to which data can be sent even while the robot is disabled
     */
    public void setEnabled(boolean enabled) {this.enabled = enabled;}
}
