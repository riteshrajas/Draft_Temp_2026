package frc.robot.utils.advancedcontrollers;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Represents a single action to be performed.  It can encapsulate any Runnable task.
 */
public class Action<T> {

    private final Consumer<T> command;
    private final T input;

    public Action(Consumer<T> command, T input) {
        this.command = command;
        this.input = input;
    }

    /**
     * Executes the action's command.
     */
    public void run(T input) {
        if (command != null) {
            command.accept(input);
        }
    }

    /**
     * Creates an Action that sets a value.
     * @param setter The consumer that sets the value.
     * @param value The value to set.
     * @return An Action that performs the set operation.
     */
    public static <T> Action<T> set(Consumer<T> setter, T value) {
        return new Action<>(setter, value);
    }

    /**
     * Creates an Action that gets a value from a supplier.
     * @param setter The consumer that sets the value.
     * @param getter The supplier that gets the value.
     * @return An Action that performs the get and set operation.
     */
    public static <T> Action<T> supply(Consumer<T> setter, Supplier<T> getter) {
        return new Action<>(setter, getter.get());
    }

    /**
     * Represents a package of actions to be executed sequentially.
     */
    public static class ActionPackage {

        private final List<Action<?>> actions;

        public ActionPackage(Action<?>... actions) {
            this.actions = new ArrayList<>(Arrays.asList(actions));
        }

        public ActionPackage() {
            this.actions = new ArrayList<>();
        }

       /**
        * Adds an action to the package.
        * @param action The action to add.
        * @return The ActionPackage instance (for chaining).
        */
        public ActionPackage addAction(Action<?> action) {
            this.actions.add(action);
            return this;
        }

        /**
         * Adds multiple actions to the package.
         * @param actions The actions to add.
         * @return The ActionPackage instance (for chaining).
         */
        public ActionPackage addActions(Action<?>... actions) {
            this.actions.addAll(Arrays.asList(actions));
            return this;
        }

        /**
         * Gets the list of actions in this package.
         * @return The list of actions.
         */
        public List<Action<?>> getActions() {
            return actions;
        }

        /**
         * Executes all actions in the package sequentially.
         */
        public void run(double input) {
            for (Action<?> action : actions) {
                ((Action<Double>) action).run(input);
            }
        }

        @Override
        public String toString() {
            StringBuilder sb = new StringBuilder("ActionPackage{");
            for (Action<?> action : actions) {
                sb.append(action.toString()).append(", ");
            }
            if (!actions.isEmpty()) {
                sb.delete(sb.length() - 2, sb.length()); // Remove trailing ", "
            }
            sb.append("}");
            return sb.toString();
        }
    }
}