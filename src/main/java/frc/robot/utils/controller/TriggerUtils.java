package frc.robot.utils.controller;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Timer;

public class TriggerUtils {
    
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

    public static Trigger debounce(Trigger trigger, double debounceTime) {
        return new Trigger(new DebouncedTrigger(trigger, debounceTime));
    }

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
                lastTrueTime = currentTime;
            }

            lastState = currentState;
            return currentState && (currentTime - lastTrueTime >= debounceTime);
        }
    }
}
