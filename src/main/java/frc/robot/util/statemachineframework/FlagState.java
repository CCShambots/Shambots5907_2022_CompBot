package frc.robot.util.statemachineframework;

import java.util.function.BooleanSupplier;

public class FlagState<E extends Enum<E>> {
    private E state;
    private BooleanSupplier condition;

    public FlagState(E state, BooleanSupplier condition) {
        this.state = state;
        this.condition = condition;
    }

    public E getState() {
        return state;
    }

    public BooleanSupplier getCondition() {
        return condition;
    }
}
