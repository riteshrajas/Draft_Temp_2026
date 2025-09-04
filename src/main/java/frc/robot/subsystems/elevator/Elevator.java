package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.States.ElevatorPIDState;
import frc.robot.States.ElevatorState;
import frc.robot.States.Status;
import frc.robot.utils.advancedcontrollers.Action;
import frc.robot.utils.motion.PIDControllerWrapper;
import frc.robot.utils.sensors.SensorFilter.MovingAverage;


@SuppressWarnings("unused")
public class Elevator extends SubsystemBase{

  /*
   * Things Elevator Have
   * 
   * - Lead Motor
   * - Follow Motor
   * - CanRange Left
   * - CanRange Right
   * - Limit Switch
   */
  private final TalonFX leadMotor;
  private final TalonFX followerMotor;
  private final CANrange rangeLeft;
  private final CANrange rangeRight;
  private final DigitalInput elevatorLimitSwitch;

  /*
   * Things Elevator should do
   * - Move up
   * - Move down
   * - Stop at limit switch
   * - Reset position
   * - Move to position and hold
   */
  // private final Action<Double> moveUp;
  // private final Action<Double> moveDown;
  // private final Action<Void> stop;
  // private final Action<Void> resetPosition;
  // private final Action<Double> moveToPosition;

  /*
   * What is Needed?
   * - Control system for the elevator
   * - Feedback from the elevator's position
   * - Safety mechanisms
   * - A way to Calibrate the elevator's position
   */
  private DoubleSupplier encoderValue;
  private final MovingAverage movingAverageFilter;
  private DoubleSupplier targetHeight;
  private final PIDControllerWrapper pidController_UP;
  private final PIDControllerWrapper pidController_DOWN;
  private Status currentStatus;
  private PIDControllerWrapper pidController;

  public Elevator() {
    // Initialize all components
    leadMotor = configuredElevatorMotor(61);
    followerMotor = configuredElevatorMotor(62);
    rangeLeft = new CANrange(45);
    rangeRight = new CANrange(46);
    elevatorLimitSwitch = new DigitalInput(2);

    currentStatus = Status.IDLE;

    pidController_UP = new PIDControllerWrapper(
        "Elevator_UP",
       ElevatorPIDState.UP.getKP(),
       ElevatorPIDState.UP.getKI(),
       ElevatorPIDState.UP.getKD(),
       ElevatorPIDState.UP.getKG(),
       ElevatorPIDState.UP.getKS());
    pidController_UP.setTolerance(0.2);
    pidController_DOWN = new PIDControllerWrapper(
        "Elevator_DOWN",
        ElevatorPIDState.DOWN.getKP(),
        ElevatorPIDState.DOWN.getKI(),
        ElevatorPIDState.DOWN.getKD(),
        ElevatorPIDState.DOWN.getKG(),
        ElevatorPIDState.DOWN.getKS());
    pidController_DOWN.setTolerance(0.2);

    // Group lead and follower motors
    followerMotor.setControl(
        new Follower(
            leadMotor.getDeviceID(),
            false));

    // Initialize actions
    // moveUp = new Action<>((Double speed) -> {
    // if (!elevatorLimitSwitch.get()) {
    // leadMotor.setControl(new DutyCycleOut(speed));
    // } else {
    // leadMotor.setControl(new NeutralOut());
    // }
    // }, 0.5);
    // moveDown = new Action<>((Double speed) -> {
    // if (!elevatorLimitSwitch.get()) {
    // leadMotor.setControl(new DutyCycleOut(-speed));
    // } else {
    // leadMotor.setControl(new NeutralOut());
    // }
    // }, -0.5);

    // Initialize MovingAverage filter for encoder readings
    movingAverageFilter = new MovingAverage(5);

  }

  @Override
  public void periodic() {
    // Update the encoder value with the filtered reading
    double rawPosition = leadMotor.getPosition().getValueAsDouble();
    double currentHeight = movingAverageFilter.calculate(rawPosition);
    encoderValue = () -> currentHeight;

    if (targetHeight == null) {
      // No target -> hold current position
      targetHeight = () -> currentHeight;
    }

    double target = targetHeight.getAsDouble();

    // Choose appropriate PID controller based on direction
    pidController = (currentHeight < target) ? pidController_UP : pidController_DOWN;
    double output = pidController.calculate(currentHeight, target);

    // Safety: Clamp output if at bottom limit and trying to go down
    if (elevatorLimitSwitch.get() && output < 0) {
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
      leadMotor.setControl(new NeutralOut());
      if (currentStatus != Status.LIMIT && currentStatus != Status.HOLDING) {
        currentStatus = Status.IDLE;
      }
    } else {
      leadMotor.setControl(new DutyCycleOut(output));
      currentStatus = (output > 0) ? Status.MOVING_UP : Status.MOVING_DOWN;
    }
  }

  public final TalonFX configuredElevatorMotor(int motorID) {
    TalonFX motor = new TalonFX(motorID);
    motor.getConfigurator().apply(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(50)
            .withStatorCurrentLimitEnable(true));
    return motor;
  }


  public Action<Double> moveToHeight(double height) {
    return Action.set(
      (Double h) -> {
        synchronized (this) {
          targetHeight = () -> h;
        }
      },
      height
    );
  }

  public Action<Double> moveToHeight(ElevatorState state) {
    return Action.set(
      (Double h) -> {
        synchronized (this) { // Locks the entire Elevator Instance
          targetHeight = () -> h;
        }
      },
      state.getHeight()
    );
  }


  public boolean atTarget() {
    if (targetHeight == null || encoderValue == null) return true;
    PIDControllerWrapper pid = (encoderValue.getAsDouble() < targetHeight.getAsDouble()) ? pidController_UP : pidController_DOWN;
    return pid.atSetpoint();
  }

  public double getCurrentHeight() {
    return encoderValue != null ? encoderValue.getAsDouble() : 0;
  }

  public void enableTuning(boolean enabled) {
    pidController_UP.enableTuning(enabled);
    pidController_DOWN.enableTuning(enabled);
  }

  public Status getCurrentStatus() {
    return currentStatus;
  }

  public void resetPosition() {
    leadMotor.setPosition(0);
  }
}
