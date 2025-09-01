# Advanced Controllers Package - Production Ready

## Overview

This package provides production-ready controllers for FRC robotics applications, specifically designed for polynomial-based trajectory following and control systems.

## Classes

### CurveKit

**Purpose**: A robust, production-ready controller that combines polynomial trajectory generation with PID control.

**Features**:
- Two operation modes: FEEDFORWARD and SETPOINT_TRACKING
- Input validation and error handling
- Output clamping for safety
- Comprehensive API with getters and setters
- Thread-safe design
- Modern Java features (switch expressions)

**Usage Examples**:

#### Feedforward Mode
```java
// Create polynomial y = 0.5x² + x
Polynomial trajectory = new Polynomial(0, 1, 0.5);

// Create feedforward controller
CurveKit feedforward = new CurveKit(
    1.0, 0.0, 0.0,              // P, I, D gains
    trajectory,                   // Polynomial
    10.0,                        // Max output
    CurveKit.Mode.FEEDFORWARD    // Mode
);

// Set up actions
Action.ActionPackage actions = new Action.ActionPackage(
    Action.set(value -> motor.set(value), 0.0)
);
feedforward.setActionPackage(actions);

// Use it
feedforward.run(timeSeconds);  // Outputs polynomial value directly
```

#### Setpoint Tracking Mode
```java
// Create polynomial for setpoint generation
Polynomial setpointGen = new Polynomial(1, 2);  // y = 1 + 2x

// Create PID controller
CurveKit pidController = new CurveKit(
    0.5, 0.01, 0.05,                    // P, I, D gains
    setpointGen,                        // Polynomial
    5.0,                               // Max output
    CurveKit.Mode.SETPOINT_TRACKING    // Mode
);

// Set up actions
Action.ActionPackage actions = new Action.ActionPackage(
    Action.set(value -> motor.set(value), 0.0)
);
pidController.setActionPackage(actions);

// Use it
double currentPosition = encoder.getPosition();
pidController.runWithMeasurement(timeSeconds, currentPosition);
```

### Action and ActionPackage

**Purpose**: Flexible action execution system for controller outputs.

**Features**:
- Type-safe action creation
- Chaining support
- Static factory methods for common operations

**Usage Examples**:
```java
// Single action
Action<Double> motorAction = Action.set(value -> motor.set(value), 0.0);

// Multiple actions
Action.ActionPackage package = new Action.ActionPackage(
    Action.set(value -> motor1.set(value), 0.0),
    Action.set(value -> motor2.set(value * 0.8), 0.0),
    Action.set(value -> SmartDashboard.putNumber("Output", value), 0.0)
);
```

### Polynomial

**Purpose**: Mathematical polynomial representation for trajectory generation.

**Usage**:
```java
// For y = ax² + bx + c, use new Polynomial(c, b, a)
Polynomial quadratic = new Polynomial(1, 2, 0.5);  // y = 1 + 2x + 0.5x²
double result = quadratic.evaluate(2.0);  // Evaluates at x = 2
```

## Error Handling

The production-ready implementation includes comprehensive error handling:

- **Constructor validation**: Null checks, positive value validation
- **Mode enforcement**: Prevents incorrect method usage for each mode
- **Runtime safety**: Output clamping, null pointer protection

## Testing

Run the production test suite:
```bash
./gradlew runProductionTest
```

This will test:
- Both operation modes
- Error handling
- Utility methods
- Input validation

## Migration from Original Demo

If migrating from the original demo:

1. **Fix polynomial coefficients**: 
   - Old: `new Polynomial(0.5, 1, 0)` → produces `0.5 + x`
   - New: `new Polynomial(0, 1, 0.5)` → produces `0.5x² + x`

2. **Choose appropriate mode**:
   - For direct output: Use `Mode.FEEDFORWARD` with `run(x)`
   - For PID control: Use `Mode.SETPOINT_TRACKING` with `runWithMeasurement(x, measurement)`

3. **Update method calls**:
   - Old: `curveKit.run(x)` (ambiguous behavior)
   - New: `curveKit.run(x)` for feedforward OR `curveKit.runWithMeasurement(x, measurement)` for PID

## Best Practices

1. **Always specify mode explicitly** in constructors
2. **Use runWithMeasurement()** for proper PID control
3. **Validate inputs** before passing to controllers
4. **Test with production test suite** before deployment
5. **Use appropriate max output values** for your actuators

## Thread Safety

The CurveKit class is designed to be thread-safe for typical FRC usage patterns. However, avoid concurrent modification of the same instance from multiple threads.
