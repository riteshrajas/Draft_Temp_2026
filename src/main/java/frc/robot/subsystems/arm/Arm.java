package frc.robot.subsystems.arm;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.States;
import frc.robot.States.ArmState;
import frc.robot.States.Status;
import frc.robot.utils.advancedcontrollers.Action;
import frc.robot.utils.motion.PIDControllerWrapper;
import frc.robot.utils.sensors.SensorFilter;

public final class Arm extends SubsystemBase {

    /*
     * Things Arm Have
     * 
     * - Pivot Motor
     * - Encoder (on motor)
     */
    private final TalonFX pivotMotor;

    /*
     * Things Arm should do
     * - Move to position (preset positions)
     * - Move at variable speeds
     * - Hold position
     * - Manual control
     */
    private SensorFilter.MovingAverage pivotAngleFilter;

    /*
     * What is Needed?
     * - Control system for the arm
     * - Feedback from the arm's position
     * - Safety mechanisms
     * - A way to Calibrate the arm's position
     */
    private DoubleSupplier targetAngle = null; // Target angle in units of motor position
    private DoubleSupplier encoderValue = () -> 0.0; // Current angle in units of motor position
    private Status currentStatus = Status.IDLE;
    private PIDControllerWrapper pidController;

    public Arm() {
        pivotMotor = configuredArmMotor(71);
        pivotAngleFilter = new SensorFilter.MovingAverage(5);
    }

    @Override
    public void periodic() {
        // Update the encoder value with the filtered reading
        double rawPosition = pivotMotor.getPosition().getValueAsDouble();
        double currentAngle = pivotAngleFilter.calculate(rawPosition);
        encoderValue = () -> currentAngle;

        if (targetAngle == null) {
            // No target -> hold current position
            targetAngle = () -> currentAngle;
        }

        double target = targetAngle.getAsDouble();

        double output = pidController.calculate(currentAngle, target);

        // Safety: Add basic angle limits (you can add limit switches later)
        if (currentAngle < 0.1 && output < 0) { // At bottom limit
            output = 0;
            currentStatus = Status.LIMIT;
        } else if (currentAngle > 0.9 && output > 0) { // At top limit
            output = 0;
            currentStatus = Status.LIMIT;
        }
        // Check if we're at target and holding position
        else if (atTarget() && Math.abs(output) < 0.05) { // Small threshold for "effectively zero"
            currentStatus = Status.HOLDING;
            output = 0; // Small output for holding
        }
        // Apply output to motor
        if (output == 0) {
            pivotMotor.setControl(new NeutralOut());
            if (currentStatus != Status.LIMIT && currentStatus != Status.HOLDING) {
                currentStatus = Status.IDLE;
            }
        } else {
            pivotMotor.setControl(new DutyCycleOut(output));
            currentStatus = (output > 0) ? Status.MOVING_UP : Status.MOVING_DOWN;
        }

        // Telemetry
        SmartDashboard.putNumber("Arm/CurrentAngle", currentAngle);
        SmartDashboard.putNumber("Arm/TargetAngle", target);
        SmartDashboard.putNumber("Arm/MotorOutput", output);
        SmartDashboard.putString("Arm/Status", currentStatus.toString());
        SmartDashboard.putBoolean("Arm/AtTarget", atTarget());
    }

    public TalonFX configuredArmMotor(int motorID) {
        TalonFX motor = new TalonFX(motorID);
        motor.getConfigurator().apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(50)
                        .withStatorCurrentLimitEnable(true));
        return motor;
    }

    public Action<Double> moveToAngle(double angle) {
        return Action.set(
                (Double a) -> {
                    synchronized (this) {
                        targetAngle = () -> a;
                        pidController = States.ArmState.Algea.getPIDState().getPIDController();
                    }
                },
                angle);
    }

    public Action<Double> moveToAngle(ArmState state) {
        return Action.set(
                (Double a) -> {
                    synchronized (this) { // Locks the entire Arm Instance
                        targetAngle = () -> a;
                        pidController = state.getPIDState().getPIDController();
                    }
                },
                state.getPosition());
    }

    public boolean atTarget() {
        if (targetAngle == null || encoderValue == null)
            return true;
        return pidController.atSetpoint();
    }

    public double getCurrentAngle() {
        return encoderValue != null ? encoderValue.getAsDouble() : 0;
    }

    public void enableTuning(boolean enabled) {
        pidController.enableTuning(enabled);
    }

    public Status getCurrentStatus() {
        return currentStatus;
    }

    public void resetPosition() {
        pivotMotor.setPosition(0);
    }

    // Manual control methods
    public void setSpeed(double speed) {
        // Override PID control for manual operation
        targetAngle = null; // Clear target to allow manual control
        pivotMotor.setControl(new DutyCycleOut(speed));
        currentStatus = Status.MOVING;
    }

    public void stop() {
        pivotMotor.setControl(new NeutralOut());
        currentStatus = Status.IDLE;
    }

    // Safety methods
    public boolean isAtLimit() {
        double currentAngle = getCurrentAngle();
        return currentAngle < 0.1 || currentAngle > 0.9;
    }
}
