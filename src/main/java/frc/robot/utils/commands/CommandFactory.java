package frc.robot.utils.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import java.util.function.BooleanSupplier;

public class CommandFactory {
    
    public static Command withTimeout(Command command, double timeoutSeconds) {
        return command.withTimeout(timeoutSeconds);
    }

    public static Command repeat(Command command, int count) {
        return new SequentialCommandGroup(
            new RepeatCommand(command).withTimeout(count * 0.02) // Estimate based on scheduler rate
        );
    }

    public static Command waitThen(double seconds, Command command) {
        return new SequentialCommandGroup(
            new WaitCommand(seconds),
            command
        );
    }

    public static Command conditional(BooleanSupplier condition, Command onTrue, Command onFalse) {
        return new ConditionalCommand(condition, onTrue, onFalse);
    }

    public static Command sequence(Command... commands) {
        return new SequentialCommandGroup(commands);
    }

    public static Command parallel(Command... commands) {
        return new ParallelCommandGroup(commands);
    }

    public static Command runOnce(Runnable action) {
        return new InstantCommand(action);
    }

    public static Command print(String message) {
        return new InstantCommand(() -> System.out.println(message));
    }

    private static class ConditionalCommand extends Command {
        private final BooleanSupplier condition;
        private final Command onTrue;
        private final Command onFalse;
        private Command selectedCommand;

        public ConditionalCommand(BooleanSupplier condition, Command onTrue, Command onFalse) {
            this.condition = condition;
            this.onTrue = onTrue;
            this.onFalse = onFalse;
        }

        @Override
        public void initialize() {
            selectedCommand = condition.getAsBoolean() ? onTrue : onFalse;
            selectedCommand.initialize();
        }

        @Override
        public void execute() {
            selectedCommand.execute();
        }

        @Override
        public boolean isFinished() {
            return selectedCommand.isFinished();
        }

        @Override
        public void end(boolean interrupted) {
            selectedCommand.end(interrupted);
        }
    }
}
