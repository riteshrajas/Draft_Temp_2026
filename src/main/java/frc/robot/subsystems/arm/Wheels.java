package frc.robot.subsystems.arm;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.States.Status;
import frc.robot.States.WheelsState;
import frc.robot.utils.advancedcontrollers.Action;
import frc.robot.utils.sensors.SensorFilter.MovingAverage;

public class Wheels extends SubsystemBase {

    /*
     * Things Wheels Have
     *
     * - Wheel Motor
     * - CANrange (Detect Coral)
     */
    private final TalonFX wheelMotor;
    private final CANrange canRange;

    /*
     * Things Wheels should do
     * - Run In (intake) - INTAKE_SLOW, INTAKE_NORMAL, INTAKE_FAST
     * - Run Out (outtake) - OUTTAKE_SLOW, OUTTAKE_NORMAL, OUTTAKE_FAST
     * - Stop - STOP
     * - Hold coral - HOLD
     * - Detect Coral
     * - Run at variable speeds
     */
    private final MovingAverage distanceFilter;
    private Status currentStatus;
    private double currentSpeed;
    private boolean coralDetected;
    private double filteredDistance;

    public Wheels() {
        // Initialize components
        wheelMotor = configuredWheelMotor(71);
        canRange = new CANrange(73);

        currentStatus = Status.IDLE;
        currentSpeed = 0.0;
        coralDetected = false;

        // Initialize sensor filter
        distanceFilter = new MovingAverage(5);
    }

    @Override
    public void periodic() {
        // Update sensor readings with filtering
        double rawDistance = canRange.getDistance().getValueAsDouble();
        filteredDistance = distanceFilter.calculate(rawDistance);

        // Update coral detection (150mm threshold)
        coralDetected = filteredDistance < 0.15;

        // Update status based on current speed
        if (Math.abs(currentSpeed) < 0.01) {
            currentStatus = Status.IDLE;
        } else if (currentSpeed > 0) {
            currentStatus = Status.MOVING;
        } else {
            currentStatus = Status.MOVING;
        }

        // Publish telemetry
        SmartDashboard.putNumber("Wheels/Speed", currentSpeed);
        SmartDashboard.putBoolean("Wheels/CoralDetected", coralDetected);
        SmartDashboard.putNumber("Wheels/Distance", filteredDistance);
        SmartDashboard.putString("Wheels/Status", currentStatus.toString());
        SmartDashboard.putString("Wheels/CurrentState", getCurrentState().toString());
    }

    public final TalonFX configuredWheelMotor(int motorID) {
        TalonFX motor = new TalonFX(motorID);
        motor.getConfigurator().apply(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(50)
                .withStatorCurrentLimitEnable(true)
        );
        return motor;
    }

    // Action-based control methods
    public Action<Double> runAtSpeed(double speed) {
        return Action.set(this::setSpeed, speed);
    }

    public Action<WheelsState> runAtState(WheelsState state) {
        return Action.set(this::setState, state);
    }

    public Action<Void> stop() {
        return Action.from(this::stopWheels);
    }

    // Basic control methods
    public void setSpeed(double speed) {
        currentSpeed = speed;
        wheelMotor.set(speed);
    }

    public void setState(WheelsState state) {
        setSpeed(state.getSpeed());
    }

    public void stopWheels() {
        setState(WheelsState.STOP);
    }

    // Sensor methods
    public boolean hasCoral() {
        return coralDetected;
    }

    public double getFilteredDistance() {
        return filteredDistance;
    }

    // Status methods
    public Status getCurrentStatus() {
        return currentStatus;
    }

    public double getCurrentSpeed() {
        return currentSpeed;
    }

    public WheelsState getCurrentState() {
        for (WheelsState state : WheelsState.values()) {
            if (Math.abs(currentSpeed - state.getSpeed()) < 0.01) {
                return state;
            }
        }
        return WheelsState.STOP;
    }


    public boolean isHolding() {
        return getCurrentState() == WheelsState.HOLD;
    }

    public boolean isSafeToRunIn() {
        return !coralDetected; // Don't intake if we already have coral
    }

    public boolean isSafeToRunOut() {
        return true; // Always safe to outtake
    }

    public boolean isSafeToRunState(WheelsState state) {
        if (state.isIntake()) {
            return isSafeToRunIn();
        } else if (state.isOuttake()) {
            return isSafeToRunOut();
        }
        return true; // STOP and HOLD are always safe
    }

    // State-based convenience methods
    public void runState(WheelsState state) {
        if (isSafeToRunState(state)) {
            setState(state);
        }
    }


    // Utility methods
    public void resetSensorFilter() {
        distanceFilter.reset();
    }


    
}