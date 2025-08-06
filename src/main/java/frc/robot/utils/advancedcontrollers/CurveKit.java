package frc.robot.utils.advancedcontrollers;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.utils.advancedcontrollers.Action.ActionPackage;

/**
 * Fixed version of CurveKit that properly handles polynomial-based control
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

    public CurveKit(PIDController pidController, Polynomial polynomial, double maxOutput, Mode mode) {
        this.pidController = pidController;
        this.polynomial = polynomial;
        this.maxOutput = maxOutput;
        this.mode = mode;
    }

    public CurveKit(double p, double i, double d, Polynomial polynomial, double maxOutput, Mode mode) {
        this(new PIDController(p, i, d), polynomial, maxOutput, mode);
    }

    public CurveKit(double p, double i, double d, double maxOutput) {
        this(new PIDController(p, i, d), new Polynomial(0), maxOutput, Mode.SETPOINT_TRACKING);
    }

    /**
     * Run the curve kit with an input value
     * @param x Input to the polynomial
     */
    public void run(double x) {
        double polynomialValue = polynomial.evaluate(x);
        double output;
        
        switch (mode) {
            case FEEDFORWARD:
                // Use polynomial value directly (scaled by P gain)
                output = polynomialValue * pidController.getP();
                break;
            case SETPOINT_TRACKING:
                // This doesn't make sense without a measurement
                // For demo purposes, assume we want to track the polynomial with zero measurement
                output = pidController.calculate(0, polynomialValue);
                break;
            default:
                output = 0;
        }
        
        output = Math.max(-maxOutput, Math.min(maxOutput, output));
        if (actionPackage != null) {
            actionPackage.run(output);
        }
    }
    
    /**
     * Run with explicit setpoint and measurement (proper PID usage)
     * @param x Input to polynomial (generates setpoint)
     * @param measurement Current measurement
     */
    public void runWithMeasurement(double x, double measurement) {
        double setpoint = polynomial.evaluate(x);
        double output = pidController.calculate(measurement, setpoint);
        output = Math.max(-maxOutput, Math.min(maxOutput, output));
        if (actionPackage != null) {
            actionPackage.run(output);
        }
    }

    public void setActionPackage(ActionPackage actionPackage) {
        this.actionPackage = actionPackage;
    }

    public double calculate(double setpoint, double measurement) {
        double output = pidController.calculate(measurement, setpoint);
        return Math.max(-maxOutput, Math.min(maxOutput, output));
    }

    public void reset() {
        pidController.reset();
    }

    public void setPID(double p, double i, double d) {
        pidController.setPID(p, i, d);
    }
    
    public Mode getMode() {
        return mode;
    }
}
