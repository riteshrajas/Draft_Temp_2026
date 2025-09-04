package frc.robot.utils.advancedcontrollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A command that wraps any Action, allowing it to be scheduled and used
 * with button bindings. This command executes the action once and then finishes.
 */
public class ActionCommand extends Command {

    private final Action<?> action;

    /**
     * Creates a Command that wraps an Action. The command will run once and finish.
     * @param action The Action to wrap. Can be of any type (e.g., Action<Double>, Action<Void>).
     * @param requirements The subsystems this action requires.
     */
    public ActionCommand(Action<?> action, Subsystem... requirements) {
        this.action = action;
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        // When the command starts, simply execute the action.
        if (action != null) {
            action.run();
        }
    }

    @Override
    public boolean isFinished() {
        // This is an "instant" command that finishes immediately after running once.
        return true;
    }
}

