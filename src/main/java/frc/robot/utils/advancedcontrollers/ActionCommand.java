package frc.robot.utils.advancedcontrollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Wrapper that makes Action<T> compatible with WPILib's Command system.
 * This allows Actions to be used with button bindings and command scheduling.
 */
public class ActionCommand extends Command {
    private final Action<Void> action;
    private final boolean continuous;
    
    /**
     * Creates a Command that wraps an Action<Void>
     * @param action The Action to wrap
     * @param continuous If true, keeps running; if false, runs once and finishes
     * @param requirements Subsystems required by this command
     */
    public ActionCommand(Action<Void> action, boolean continuous, Subsystem... requirements) {
        this.action = action;
        this.continuous = continuous;
        addRequirements(requirements);
    }
    
    /**
     * Creates a Command that wraps an Action<Void> (instant by default)
     * @param action The Action to wrap
     * @param requirements Subsystems required by this command
     */
    public ActionCommand(Action<Void> action, Subsystem... requirements) {
        this(action, false, requirements);
    }
    
    @Override
    public void initialize() {
        // Execute the action when the command starts
        if (action != null) {
            action.run(null);
        }
    }
    
    @Override
    public void execute() {
        // For continuous actions, keep running the action
        if (continuous && action != null) {
            action.run(null);
        }
    }
    
    @Override
    public boolean isFinished() {
        // Continuous actions run until cancelled, instant actions finish immediately
        return !continuous;
    }
    
    /**
     * Static factory method for instant actions
     * @param action The Action to wrap
     * @param requirements Subsystems required
     * @return A new ActionCommand that runs once
     */
    public static ActionCommand from(Action<Void> action, Subsystem... requirements) {
        return new ActionCommand(action, false, requirements);
    }
    
    /**
     * Static factory method for continuous actions
     * @param action The Action to wrap
     * @param requirements Subsystems required
     * @return A new ActionCommand that runs continuously until cancelled
     */
    public static ActionCommand continuous(Action<Void> action, Subsystem... requirements) {
        return new ActionCommand(action, true, requirements);
    }
}
