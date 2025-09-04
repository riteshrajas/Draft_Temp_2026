package frc.robot.utils.advancedcontrollers;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Represents a single, reusable robot action.
 * It cleanly separates the definition of WHAT to do (e.g., set a motor speed)
 * from the execution of that action.
 * This version is architected to be intuitive and avoid side effects.
 */
public class Action<T> {

    private final Consumer<T> command;
    private final Supplier<T> valueSupplier;
    

    /**
     * Private constructor. Use the static factory methods 'set', 'supply', or 'from' instead.
     */
    private Action(Consumer<T> command, Supplier<T> valueSupplier) {
        this.command = command;
        this.valueSupplier = valueSupplier;
    }

    /**
     * Executes the action's command with its internally stored value.
     * This method takes no parameters, making it simple to call from commands.
     */
    public void run() {
        if (command != null && valueSupplier != null) {
            command.accept(valueSupplier.get());
        }
    }

    /**
     * Creates an Action that sets a fixed, predetermined value.
     * @param setter The method that sets the value (e.g., elevator::setTargetHeight).
     * @param value The fixed value to set.
     * @return A new Action that performs the set operation.
     */
    public static <T> Action<T> set(Consumer<T> setter, T value) {
        return new Action<>(setter, () -> value);
    }

    /**
     * Creates an Action that gets a value from a supplier at runtime.
     * @param setter The method that sets the value.
     * @param getter The supplier that provides the value when the action is run.
     * @return A new Action that performs the get-and-set operation.
     */
    public static <T> Action<T> supply(Consumer<T> setter, Supplier<T> getter) {
        return new Action<>(setter, getter);
    }

    /**
     * Creates an Action from a simple Runnable for tasks that don't require a value.
     * @param task The Runnable to execute when the action is run.
     * @return A new Action<Void> that executes the task.
     */
    public static Action<Void> from(Runnable task) {
        // The consumer ignores its input and just runs the task.
        // The supplier provides a null of type Void.
        return new Action<>(v -> task.run(), () -> null);
    }

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
            // for (Action<?> action : actions) {
            //     ((Action<Double>) action).run(input);
            // }
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

