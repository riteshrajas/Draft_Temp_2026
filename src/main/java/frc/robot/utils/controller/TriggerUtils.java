package frc.robot.utils.controller;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Small utility helpers for composing and modifying {@link Trigger} objects.
 *
 * <p>Provides common boolean compositions and a debouncing helper useful for
 * controller button events. All helpers return a new {@link Trigger} instance
 * that can be used with the WPILib command-based API.</p>
 *
 * Provided utilities:
 * <ul>
 *   <li>{@link #and(Trigger...)}</li>
 *   <li>{@link #or(Trigger...)}</li>
 *   <li>{@link #debounce(Trigger, double)}</li>
 * </ul>
 */
public class TriggerUtils {
    
    /**
     * Logical AND composition of multiple triggers.
     *
     * @param triggers one or more triggers to AND together
     * @return a new Trigger that evaluates true only when every provided trigger is true
     */
    public static Trigger and(Trigger... triggers) {
        return new Trigger(() -> {
            for (Trigger trigger : triggers) {
                if (!trigger.getAsBoolean()) {
                    return false;
                }
            }
            return true;
        });
    }

    /**
     * Logical OR composition of multiple triggers.
     *
     * @param triggers one or more triggers to OR together
     * @return a new Trigger that evaluates true if any provided trigger is true
     */
    public static Trigger or(Trigger... triggers) {
        return new Trigger(() -> {
            for (Trigger trigger : triggers) {
                if (trigger.getAsBoolean()) {
                    return true;
                }
            }
            return false;
        });
    }

    /**
     * Debounce a trigger so it only reports true after the underlying trigger has
     * been continuously true for at least {@code debounceTime} seconds.
     *
     * <p>Typical use: remove contact-bounce or rapid button chatter by wrapping a
     * physical button trigger with this debouncer.</p>
     *
     * @param trigger the underlying trigger to debounce
     * @param debounceTime time in seconds the trigger must remain continuously true before
     *                     the returned trigger reports true
     * @return a new Trigger that applies debounce timing
     */
    public static Trigger debounce(Trigger trigger, double debounceTime) {
        return new Trigger(new DebouncedTrigger(trigger, debounceTime));
    }

    /**
     * Internal BooleanSupplier that implements time-based debouncing.
     *
     * Behavior notes / edge cases:
     * <ul>
     *   <li>When the underlying trigger transitions from false -> true we record the
     *       first time it became true and only return true after {@code debounceTime}
     *       seconds have elapsed while the underlying trigger remains true.</li>
     *   <li>If the underlying trigger falls false the debouncer resets and the
     *       next rising edge restarts the timing window.</li>
     *   <li>Uses {@link Timer#getFPGATimestamp()} for monotonic FPGA time.
     *   </li>
     * </ul>
     */
    private static class DebouncedTrigger implements java.util.function.BooleanSupplier {
        private final Trigger trigger;
        private final double debounceTime;
        private double lastTrueTime = 0;
        private boolean lastState = false;

        public DebouncedTrigger(Trigger trigger, double debounceTime) {
            this.trigger = trigger;
            this.debounceTime = debounceTime;
        }

        @Override
        public boolean getAsBoolean() {
            boolean currentState = trigger.getAsBoolean();
            double currentTime = Timer.getFPGATimestamp();

            if (currentState && !lastState) {
                // Rising edge: start debounce timer
                lastTrueTime = currentTime;
            }

            lastState = currentState;
            return currentState && (currentTime - lastTrueTime >= debounceTime);
        }
    }
}
