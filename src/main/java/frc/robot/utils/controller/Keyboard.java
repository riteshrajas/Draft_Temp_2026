package frc.robot.utils.controller;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

import javax.swing.JFrame;
import javax.swing.JTextField;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Keyboard input handler for testing commands during simulation.
 * Only active when running in simulation mode.
 */
public class Keyboard {

    private static JFrame frame;
    private static JTextField textField;
    private static Map<Character, Runnable> keyBindings = new HashMap<>();
    private static boolean initialized = false;

    /**
     * Initialize the keyboard listener. Call this once in robotInit().
     */
    public static void initialize() {
        if (!RobotBase.isSimulation() || initialized) {
            return;
        }

        frame = new JFrame("Robot Keyboard Control");
        frame.setDefaultCloseOperation(JFrame.HIDE_ON_CLOSE);
        frame.setSize(300, 100);
        frame.setAlwaysOnTop(true);

        textField = new JTextField();
        textField.addKeyListener(new KeyListener() {
            @Override
            public void keyTyped(KeyEvent e) {
                char key = Character.toLowerCase(e.getKeyChar());
                if (keyBindings.containsKey(key)) {
                    keyBindings.get(key).run();
                }
            }

            @Override
            public void keyPressed(KeyEvent e) {}

            @Override
            public void keyReleased(KeyEvent e) {}
        });

        frame.add(textField);
        frame.setVisible(true);
        initialized = true;

        DriverStation.reportWarning("Keyboard control initialized. Click the text field and press keys to trigger commands.", false);
    }

    /**
     * Bind a key to a command/action.
     * @param key The character key to bind (case insensitive)
     * @param action The action to run when the key is pressed
     */
    public static void bindKey(char key, Runnable action) {
        keyBindings.put(Character.toLowerCase(key), action);
    }

    /**
     * Bind a key to an action that takes a parameter.
     * @param key The character key to bind (case insensitive)
     * @param action The action to run when the key is pressed
     * @param parameter The parameter to pass to the action
     */
    public static <T> void bindKey(char key, Consumer<T> action, T parameter) {
        keyBindings.put(Character.toLowerCase(key), () -> action.accept(parameter));
    }

    /**
     * Remove a key binding.
     * @param key The key to unbind
     */
    public static void unbindKey(char key) {
        keyBindings.remove(Character.toLowerCase(key));
    }

    /**
     * Show the keyboard control window.
     */
    public static void show() {
        if (frame != null) {
            frame.setVisible(true);
            frame.toFront();
        }
    }

    /**
     * Hide the keyboard control window.
     */
    public static void hide() {
        if (frame != null) {
            frame.setVisible(false);
        }
    }

    /**
     * Check if keyboard control is active.
     * @return true if running in simulation and initialized
     */
    public static boolean isActive() {
        return RobotBase.isSimulation() && initialized;
    }

    /**
     * Get a list of all bound keys.
     * @return String describing bound keys
     */
    public static String getBoundKeys() {
        if (keyBindings.isEmpty()) {
            return "No keys bound";
        }

        StringBuilder sb = new StringBuilder("Bound keys: ");
        for (char key : keyBindings.keySet()) {
            sb.append(key).append(", ");
        }
        return sb.substring(0, sb.length() - 2);
    }

    // ===== CONVENIENCE METHODS FOR COMMON ACTIONS =====

    /**
     * Bind common movement keys.
     * @param forward Action to run when 'W' is pressed
     * @param backward Action to run when 'S' is pressed
     * @param left Action to run when 'A' is pressed
     * @param right Action to run when 'D' is pressed
     * @param stop Action to run when 'Space' is pressed
     */
    public static void setupMovementKeys(Runnable forward, Runnable backward,
                                       Runnable left, Runnable right, Runnable stop) {
        bindKey('w', forward);
        bindKey('s', backward);
        bindKey('a', left);
        bindKey('d', right);
        bindKey(' ', stop);
    }

    /**
     * Bind common elevator keys.
     * @param moveUp Action to run when 'W' is pressed
     * @param moveDown Action to run when 'S' is pressed
     * @param stop Action to run when 'Space' is pressed
     * @param reset Action to run when 'R' is pressed
     * @param goToZero Action to run when 'Z' is pressed
     */
    public static void setupElevatorKeys(Runnable moveUp, Runnable moveDown,
                                       Runnable stop, Runnable reset, Runnable goToZero) {
        bindKey('w', moveUp);
        bindKey('s', moveDown);
        bindKey(' ', stop);
        bindKey('r', reset);
        bindKey('z', goToZero);
    }

    /**
     * Print help information to console.
     */
    public static void printHelp() {
        System.out.println("=== Keyboard Control Help ===");
        System.out.println("Click the 'Robot Keyboard Control' window and press keys:");
        System.out.println("WASD: Movement controls");
        System.out.println("Space: Stop/Emergency stop");
        System.out.println("R: Reset position");
        System.out.println("Z: Go to zero/home position");
        System.out.println("Q/E: Rotate left/right (if bound)");
        System.out.println();
        System.out.println(getBoundKeys());
        System.out.println("============================");
    }
}
