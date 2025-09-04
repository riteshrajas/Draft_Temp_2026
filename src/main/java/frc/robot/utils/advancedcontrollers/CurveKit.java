package frc.robot.utils.advancedcontrollers;


import edu.wpi.first.math.controller.PIDController;
import frc.robot.utils.advancedcontrollers.Action.ActionPackage;

/**
 * Production-ready CurveKit for polynomial-based control systems
 * 
 * This class provides two modes of operation:
 * - FEEDFORWARD: Uses polynomial output directly (scaled by P gain)
 * - SETPOINT_TRACKING: Uses polynomial output as setpoint for PID control
 * 
 * Usage:
 * - For feedforward control: Use run(x) with FEEDFORWARD mode
 * - For PID control: Use runWithMeasurement(x, measurement) with SETPOINT_TRACKING mode
 * 
 * @author FRC Team
 * @version 1.0
 */
public class CurveKit {
    private final PIDController pidController;
    private final double maxOutput;
    private final Polynomial polynomial;
    private ActionPackage actionPackage;
    
    // Mode enum to clarify the intended usage
    public enum Mode {
        FEEDFORWARD,    // Polynomial output is used directly (with scaling)
        SETPOINT_TRACKING  // Polynomial output is used as setpoint for PID
    }
    
    private final Mode mode;

    /**
     * Creates a CurveKit with a pre-configured PID controller
     * @param pidController The PID controller to use
     * @param polynomial The polynomial for trajectory generation
     * @param maxOutput Maximum absolute output value
     * @param mode The operation mode (FEEDFORWARD or SETPOINT_TRACKING)
     * @throws IllegalArgumentException if any parameter is invalid
     */
    public CurveKit(PIDController pidController, Polynomial polynomial, double maxOutput, Mode mode) {
        if (pidController == null) {
            throw new IllegalArgumentException("PID controller cannot be null");
        }
        if (polynomial == null) {
            throw new IllegalArgumentException("Polynomial cannot be null");
        }
        if (maxOutput <= 0) {
            throw new IllegalArgumentException("Max output must be positive");
        }
        if (mode == null) {
            throw new IllegalArgumentException("Mode cannot be null");
        }
        
        this.pidController = pidController;
        this.polynomial = polynomial;
        this.maxOutput = maxOutput;
        this.mode = mode;
    }

    /**
     * Creates a CurveKit with PID constants
     * @param p Proportional gain
     * @param i Integral gain  
     * @param d Derivative gain
     * @param polynomial The polynomial for trajectory generation
     * @param maxOutput Maximum absolute output value
     * @param mode The operation mode (FEEDFORWARD or SETPOINT_TRACKING)
     * @throws IllegalArgumentException if any parameter is invalid
     */
    public CurveKit(double p, double i, double d, Polynomial polynomial, double maxOutput, Mode mode) {
        this(new PIDController(p, i, d), polynomial, maxOutput, mode);
    }

    /**
     * Creates a CurveKit with PID constants and default polynomial (backward compatibility)
     * @param p Proportional gain
     * @param i Integral gain
     * @param d Derivative gain
     * @param maxOutput Maximum absolute output value
     * @deprecated Use the constructor that explicitly specifies polynomial and mode
     */
    @Deprecated
    public CurveKit(double p, double i, double d, double maxOutput) {
        this(new PIDController(p, i, d), new Polynomial(0), maxOutput, Mode.SETPOINT_TRACKING);
    }

    /**
     * Run the curve kit with an input value
     * @param x Input to the polynomial
     * @throws IllegalStateException if mode is SETPOINT_TRACKING (use runWithMeasurement instead)
     */
    public void run(double x) {
        double polynomialValue = polynomial.evaluate(x);
        double output = switch (mode) {
            case FEEDFORWARD -> polynomialValue * pidController.getP();
            case SETPOINT_TRACKING -> throw new IllegalStateException(
                "Cannot use run(x) with SETPOINT_TRACKING mode. Use runWithMeasurement(x, measurement) instead."
            );
        };
        
        output = clampOutput(output);
        executeActions(output);
    }
    
    /**
     * Run with explicit setpoint and measurement (proper PID usage)
     * @param x Input to polynomial (generates setpoint)
     * @param measurement Current measurement
     */
    public void runWithMeasurement(double x, double measurement) {
        if (mode != Mode.SETPOINT_TRACKING) {
            throw new IllegalStateException(
                "runWithMeasurement() should only be used with SETPOINT_TRACKING mode"
            );
        }
        
        double setpoint = polynomial.evaluate(x);
        double output = pidController.calculate(measurement, setpoint);
        output = clampOutput(output);
        executeActions(output);
    }

    /**
     * Clamp output to the specified maximum
     * @param output The output value to clamp
     * @return The clamped output value
     */
    private double clampOutput(double output) {
        return Math.max(-maxOutput, Math.min(maxOutput, output));
    }
    
    /**
     * Execute the action package with the given output
     * @param output The output value to pass to actions
     */
    private void executeActions(double output) {
        if (actionPackage != null) {
            actionPackage.run(output);
        }
    }

    /**
     * Sets the action package to execute with outputs
     * @param actionPackage The action package to set (can be null to disable actions)
     */
    public void setActionPackage(ActionPackage actionPackage) {
        this.actionPackage = actionPackage;
    }

    /**
     * Gets the current action package
     * @return The current action package (may be null)
     */
    public ActionPackage getActionPackage() {
        return actionPackage;
    }

    /**
     * Calculate PID output for given setpoint and measurement
     * @param setpoint The desired setpoint
     * @param measurement The current measurement
     * @return The clamped PID output
     */
    public double calculate(double setpoint, double measurement) {
        double output = pidController.calculate(measurement, setpoint);
        return clampOutput(output);
    }

    /**
     * Reset the PID controller state
     */
    public void reset() {
        pidController.reset();
    }

    /**
     * Update PID constants
     * @param p Proportional gain
     * @param i Integral gain
     * @param d Derivative gain
     */
    public void setPID(double p, double i, double d) {
        pidController.setPID(p, i, d);
    }
    
    /**
     * Get the current operating mode
     * @return The current mode
     */
    public Mode getMode() {
        return mode;
    }
    
    /**
     * Get the maximum output value
     * @return The maximum output value
     */
    public double getMaxOutput() {
        return maxOutput;
    }
    

    public Polynomial getPolynomial() {
        return polynomial;
    }
    

    public PIDController getPIDController() {
        return pidController;
    }
}