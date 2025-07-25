package frc.robot.utils.motion;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.telemetry.TelemetryManager;

public class PIDControllerWrapper {
    private PIDController pidController;
    private String name;
    private boolean tuningEnabled = false;
    private double lastKp, lastKi, lastKd;

    public PIDControllerWrapper(String name, double kp, double ki, double kd) {
        this.name = name;
        this.pidController = new PIDController(kp, ki, kd);
        this.lastKp = kp;
        this.lastKi = ki;
        this.lastKd = kd;
        
        initializeDashboard();
    }

    private void initializeDashboard() {
        SmartDashboard.putNumber(name + "/kP", lastKp);
        SmartDashboard.putNumber(name + "/kI", lastKi);
        SmartDashboard.putNumber(name + "/kD", lastKd);
        SmartDashboard.putBoolean(name + "/TuningEnabled", tuningEnabled);
    }

    public void enableTuning(boolean enabled) {
        this.tuningEnabled = enabled;
        SmartDashboard.putBoolean(name + "/TuningEnabled", enabled);
    }

    public double calculate(double measurement, double setpoint) {
        updateFromDashboard();
        
        double output = pidController.calculate(measurement, setpoint);
        
        
        return output;
    }

    private void updateFromDashboard() {
        if (!tuningEnabled) return;

        double kp = SmartDashboard.getNumber(name + "/kP", lastKp);
        double ki = SmartDashboard.getNumber(name + "/kI", lastKi);
        double kd = SmartDashboard.getNumber(name + "/kD", lastKd);

        if (kp != lastKp || ki != lastKi || kd != lastKd) {
            pidController.setPID(kp, ki, kd);
            lastKp = kp;
            lastKi = ki;
            lastKd = kd;
        }
    }

    public void setTolerance(double tolerance) {
        pidController.setTolerance(tolerance);
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    public void reset() {
        pidController.reset();
    }
}
