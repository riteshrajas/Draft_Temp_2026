package frc.robot.utils.units;

public class UnitConverter {
    
    // Distance conversions
    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }
    
    public static double metersToInches(double meters) {
        return meters / 0.0254;
    }
    
    public static double feetToMeters(double feet) {
        return feet * 0.3048;
    }
    
    public static double metersToFeet(double meters) {
        return meters / 0.3048;
    }
    
    // Angular conversions
    public static double degreesToRadians(double degrees) {
        return Math.toRadians(degrees);
    }
    
    public static double radiansToDegrees(double radians) {
        return Math.toDegrees(radians);
    }
    
    public static double rotationsToRadians(double rotations) {
        return rotations * 2 * Math.PI;
    }
    
    public static double radiansToRotations(double radians) {
        return radians / (2 * Math.PI);
    }
    
    // Velocity conversions
    public static double rpmToRadPerSec(double rpm) {
        return rpm * 2 * Math.PI / 60;
    }
    
    public static double radPerSecToRpm(double radPerSec) {
        return radPerSec * 60 / (2 * Math.PI);
    }
    
    // Motor encoder conversions
    public static double encoderTicksToDistance(double ticks, double ticksPerRevolution, double wheelDiameter) {
        double rotations = ticks / ticksPerRevolution;
        return rotations * Math.PI * wheelDiameter;
    }
    
    public static double distanceToEncoderTicks(double distance, double ticksPerRevolution, double wheelDiameter) {
        double rotations = distance / (Math.PI * wheelDiameter);
        return rotations * ticksPerRevolution;
    }
    
    // Drivetrain specific
    public static double wheelRpmToGroundSpeed(double wheelRpm, double wheelDiameter) {
        double wheelCircumference = Math.PI * wheelDiameter;
        return wheelRpm * wheelCircumference / 60; // Convert to units per second
    }
    
    public static double groundSpeedToWheelRpm(double groundSpeed, double wheelDiameter) {
        double wheelCircumference = Math.PI * wheelDiameter;
        return groundSpeed * 60 / wheelCircumference; // Convert to RPM
    }
}
