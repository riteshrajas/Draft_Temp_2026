package frc.robot.utils.motion;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Lightweight wrapper around {@link PIDController} that adds runtime tuning and
 * conveniences for use on the robot.
 *
 * <p>This wrapper does not change the underlying PID algorithm — it simply
 * provides:
 * <ul>
 *   <li>Runtime tuning via {@link SmartDashboard} (kP, kI, kD) with a toggle to
 *       enable/disable tuning.</li>
 *   <li>Grouping of SmartDashboard keys under a human-readable {@code name} to
 *       keep telemetry organized.</li>
 *   <li>A small optimization to avoid repeatedly calling {@code setPID()} unless
 *       a parameter actually changed.</li>
 *   <li>Convenience passthroughs for {@code setTolerance}, {@code atSetpoint}, and
 *       {@code reset} so callers don't need to reach into the raw controller.
 * </ul>
 *
 * Why use this wrapper instead of using {@link PIDController} directly?
 * <ul>
 *   <li>Runtime tuning: You can adjust PID gains while the robot is running and
 *       immediately observe the effect without redeploying code. This speeds up
 *       iterative tuning during testing and on the field.</li>
 *   <li>Safety and ergonomics: Tuning is opt-in via {@code enableTuning}. When
 *       disabled, the controller runs with the last known good gains and ignores
 *       dashboard changes.
 *   </li>
 *   <li>Reduced churn: The wrapper only calls {@code setPID} when values change,
 *       avoiding unnecessary internal updates and keeping dashboard reads cheap.
 *   </li>
 *   <li>Consistent telemetry: Keys are consistently named ("{name}/kP" etc.),
 *       so dashboards and logging tools can programmatically find and display
 *       controller parameters.
 *   </ul>
 *
 * Contract and edge cases:
 * <ul>
 *   <li>Inputs: {@code measurement} and {@code setpoint} are provided to
 *       {@link #calculate(double, double)}.</li>
 *   <li>Outputs: the PID controller output (double) is returned to the caller.</li>
 *   <li>Invalid constructor arguments (e.g. NaN gains) are the caller's
 *       responsibility; this thin wrapper does not validate numeric ranges.
 * </ul>
 */
public class PIDControllerWrapper {
    private final PIDController pidController;
    private final String name;
    private boolean tuningEnabled = false;
    private double lastKp, lastKi, lastKd, lastKg, lastKs;
    private double tolerance = 0;
    private ArmFeedforward feedforward;


    /**
     * Create a PID controller wrapper and publish initial values to the
     * SmartDashboard under the provided {@code name}.
     *
     * @param name human-readable name used as a SmartDashboard key prefix (e.g. "Drive/Heading")
     * @param kp initial proportional gain
     * @param ki initial integral gain
     * @param kd initial derivative gain
     */
    public PIDControllerWrapper(String name, double kp, double ki, double kd) {
        this.name = name;
        this.pidController = new PIDController(kp, ki, kd);
        this.lastKp = kp;
        this.lastKi = ki;
        this.lastKd = kd;
        this.lastKg = 0;
        this.lastKs = 0;
        this.feedforward = new ArmFeedforward(0, 0, 0, 0);

        initializeDashboard();
    }
    /**
     * Create a PID controller wrapper and publish initial values to the
     * SmartDashboard under the provided {@code name}.
     *
     * @param name human-readable name used as a SmartDashboard key prefix (e.g. "Drive/Heading")
     * @param kp initial proportional gain
     * @param ki initial integral gain
     * @param kd initial derivative gain
     * @param kg initial gravity gain
     * @param ks initial static gain
     */
    public PIDControllerWrapper(String name, double kp, double ki, double kd, double kg, double ks) {
        this.name = name;
        this.pidController = new PIDController(kp, ki, kd);
        this.lastKp = kp;
        this.lastKi = ki;
        this.lastKd = kd;
        this.lastKg = kg;
        this.lastKs = ks;
        this.feedforward = new ArmFeedforward(ks, kg, 0, 0);

        initializeDashboard();
    }


   


    /**
     * Publish initial values to SmartDashboard so a tuner can see and adjust them.
     * Keys created:
     * <ul>
     *   <li>{@code name + "/kP"}</li>
     *   <li>{@code name + "/kI"}</li>
     *   <li>{@code name + "/kD"}</li>
     *   <li>{@code name + "/kG"}</li>
     *   <li>{@code name + "/kS"}</li>
     *   <li>{@code name + "/Tolerance"}</li>
     *   <li>{@code name + "/TuningEnabled"}</li>
     * </ul>
     */
    private void initializeDashboard() {
        SmartDashboard.putNumber(name + "/kP", lastKp);
        SmartDashboard.putNumber(name + "/kI", lastKi);
        SmartDashboard.putNumber(name + "/kD", lastKd);
        SmartDashboard.putNumber(name + "/kG", lastKg);
        SmartDashboard.putNumber(name + "/kS", lastKs);
        SmartDashboard.putNumber(name + "/Tolerance", tolerance);
        SmartDashboard.putBoolean(name + "/TuningEnabled", tuningEnabled);
    }

    /**
     * Enable or disable runtime tuning. When disabled, dashboard changes are ignored.
     *
     * @param enabled true to allow live tuning from SmartDashboard
     */
    public void enableTuning(boolean enabled) {
        this.tuningEnabled = enabled;
        SmartDashboard.putBoolean(name + "/TuningEnabled", enabled);
    }

    /**
     * Compute the PID output for the given measurement and setpoint.
     * If tuning is enabled this method will first read the latest gains from
     * SmartDashboard and update the underlying controller if they changed.
     *
     * @param measurement current process variable
     * @param setpoint desired setpoint
     * @return controller output
     */
    public double calculate(double measurement, double setpoint) {
        updateFromDashboard();
        
        double pidOutput = pidController.calculate(measurement, setpoint);
        double ffOutput = feedforward.calculate(setpoint, 0); // Assuming setpoint is position, velocity = 0
        double output = pidOutput + ffOutput;
        
        return output;
    }

    /**
     * Read PID gains from SmartDashboard and apply them if different from the
     * last-applied values. This avoids unnecessary calls to {@code setPID}.
     */
    private void updateFromDashboard() {
        if (!tuningEnabled) return;

        double kp = SmartDashboard.getNumber(name + "/kP", lastKp);
        double ki = SmartDashboard.getNumber(name + "/kI", lastKi);
        double kd = SmartDashboard.getNumber(name + "/kD", lastKd);
        double kg = SmartDashboard.getNumber(name + "/kG", lastKg);
        double ks = SmartDashboard.getNumber(name + "/kS", lastKs);
        double tol = SmartDashboard.getNumber(name + "/Tolerance", tolerance);

        if (kp != lastKp || ki != lastKi || kd != lastKd || kg != lastKg || ks != lastKs || tol != tolerance) {
            pidController.setPID(kp, ki, kd);
            feedforward = new ArmFeedforward(ks, kg, 0, 0);
            pidController.setTolerance(tol);
            lastKp = kp;
            lastKi = ki;
            lastKd = kd;
            lastKg = kg;
            lastKs = ks;
            tolerance = tol;
        }
    }

    /**
     * Convenience passthrough to set the controller tolerance.
     *
     * @param tolerance tolerance value used by {@link PIDController#atSetpoint()}
     */
    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
        pidController.setTolerance(tolerance);
    }

    /**
     * @return true if the controller reports it is at the setpoint
     */
    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    /**
     * Reset the controller internal integrators and state.
     */
    public void reset() {
        pidController.reset();
    }
}
