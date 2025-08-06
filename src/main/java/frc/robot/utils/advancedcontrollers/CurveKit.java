package frc.robot.utils.advancedcontrollers;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.utils.advancedcontrollers.Action.ActionPackage;

public class CurveKit {
    private final PIDController pidController;
    private final double maxOutput;
    private final Polynomial polynomial;
    private ActionPackage actionPackage;

    public CurveKit(PIDController pidController, Polynomial polynomial, double maxOutput) {
        this.pidController = pidController;
        this.polynomial = polynomial;
        this.maxOutput = maxOutput;
    }



    public CurveKit(double p, double i, double d, Polynomial polynomial, double maxOutput) {
        this(new PIDController(p, i, d), polynomial, maxOutput);
    }

    public CurveKit(double p, double i, double d, double maxOutput) {
        this(new PIDController(p, i, d), new Polynomial(0), maxOutput);
    }

    public void run(double x) {
        double polynomialValue = polynomial.evaluate(x);
        double output = pidController.calculate(polynomialValue);
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
}