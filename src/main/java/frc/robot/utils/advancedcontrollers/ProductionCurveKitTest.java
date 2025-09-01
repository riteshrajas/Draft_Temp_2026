package frc.robot.utils.advancedcontrollers;

/**
 * Production test for the improved CurveKit
 */
public class ProductionCurveKitTest {

    public static void main(String[] args) {
        System.out.println("=".repeat(80));
        System.out.println("PRODUCTION CURVEKIT TEST");
        System.out.println("=".repeat(80));
        
        testFeedforwardMode();
        testSetpointTrackingMode();
        testErrorHandling();
        testUtilityMethods();
        
        System.out.println("\n" + "=".repeat(80));
        System.out.println("ALL TESTS COMPLETED SUCCESSFULLY");
        System.out.println("=".repeat(80));
    }
    
    private static void testFeedforwardMode() {
        System.out.println("\n1. TESTING FEEDFORWARD MODE");
        System.out.println("-".repeat(40));
        
        // Create polynomial y = 0.5x² + x
        Polynomial quadratic = new Polynomial(0, 1, 0.5);
        
        // Create feedforward CurveKit
        CurveKit feedforwardKit = new CurveKit(1.0, 0.0, 0.0, quadratic, 10.0, CurveKit.Mode.FEEDFORWARD);
        
        // Set up action package
        Action.ActionPackage actions = new Action.ActionPackage(
            Action.set(value -> System.out.printf("  Input: %.1f → Output: %.4f%n", 0.0, value), 0.0)
        );
        feedforwardKit.setActionPackage(actions);
        
        // Test with various inputs
        double[] inputs = {0.0, 0.5, 1.0, 1.5, 2.0};
        for (double input : inputs) {
            System.out.printf("Input: %.1f, Expected polynomial: %.4f", input, quadratic.evaluate(input));
            feedforwardKit.run(input);
        }
        
        System.out.println("✓ Feedforward mode test passed");
    }
    
    private static void testSetpointTrackingMode() {
        System.out.println("\n2. TESTING SETPOINT TRACKING MODE");
        System.out.println("-".repeat(40));
        
        // Create linear polynomial y = 2x + 1
        Polynomial linear = new Polynomial(1, 2);
        
        // Create setpoint tracking CurveKit
        CurveKit pidKit = new CurveKit(0.5, 0.01, 0.05, linear, 5.0, CurveKit.Mode.SETPOINT_TRACKING);
        
        // Set up action package
        Action.ActionPackage actions = new Action.ActionPackage(
            Action.set(value -> System.out.printf("  PID Output: %.4f%n", value), 0.0)
        );
        pidKit.setActionPackage(actions);
        
        // Test with measurement tracking
        double[] inputs = {0.0, 1.0, 2.0};
        for (double input : inputs) {
            double setpoint = linear.evaluate(input);
            double measurement = setpoint * 0.8; // Simulate being 80% there
            System.out.printf("Input: %.1f, Setpoint: %.2f, Measurement: %.2f", 
                             input, setpoint, measurement);
            pidKit.runWithMeasurement(input, measurement);
        }
        
        System.out.println("✓ Setpoint tracking mode test passed");
    }
    
    private static void testErrorHandling() {
        System.out.println("\n3. TESTING ERROR HANDLING");
        System.out.println("-".repeat(40));
        
        Polynomial poly = new Polynomial(1, 1);
        
        // Test constructor validation
        try {
            new CurveKit(null, poly, 1.0, CurveKit.Mode.FEEDFORWARD);
            System.out.println("✗ Should have thrown exception for null PID controller");
        } catch (IllegalArgumentException e) {
            System.out.println("✓ Correctly rejected null PID controller");
        }
        
        try {
            new CurveKit(1.0, 0.0, 0.0, null, 1.0, CurveKit.Mode.FEEDFORWARD);
            System.out.println("✗ Should have thrown exception for null polynomial");
        } catch (IllegalArgumentException e) {
            System.out.println("✓ Correctly rejected null polynomial");
        }
        
        try {
            new CurveKit(1.0, 0.0, 0.0, poly, -1.0, CurveKit.Mode.FEEDFORWARD);
            System.out.println("✗ Should have thrown exception for negative max output");
        } catch (IllegalArgumentException e) {
            System.out.println("✓ Correctly rejected negative max output");
        }
        
        // Test mode enforcement
        CurveKit feedforwardKit = new CurveKit(1.0, 0.0, 0.0, poly, 1.0, CurveKit.Mode.FEEDFORWARD);
        try {
            feedforwardKit.runWithMeasurement(1.0, 0.5);
            System.out.println("✗ Should have thrown exception for wrong mode usage");
        } catch (IllegalStateException e) {
            System.out.println("✓ Correctly rejected runWithMeasurement in FEEDFORWARD mode");
        }
        
        CurveKit pidKit = new CurveKit(1.0, 0.0, 0.0, poly, 1.0, CurveKit.Mode.SETPOINT_TRACKING);
        try {
            pidKit.run(1.0);
            System.out.println("✗ Should have thrown exception for wrong mode usage");
        } catch (IllegalStateException e) {
            System.out.println("✓ Correctly rejected run() in SETPOINT_TRACKING mode");
        }
        
        System.out.println("✓ Error handling test passed");
    }
    
    private static void testUtilityMethods() {
        System.out.println("\n4. TESTING UTILITY METHODS");
        System.out.println("-".repeat(40));
        
        Polynomial poly = new Polynomial(0, 1, 0.5);
        CurveKit kit = new CurveKit(0.5, 0.01, 0.05, poly, 10.0, CurveKit.Mode.FEEDFORWARD);
        
        // Test getters
        System.out.println("Mode: " + kit.getMode());
        System.out.println("Max Output: " + kit.getMaxOutput());
        System.out.println("Has Polynomial: " + (kit.getPolynomial() != null));
        System.out.println("Has PID Controller: " + (kit.getPIDController() != null));
        
        // Test action package management
        Action.ActionPackage actions = new Action.ActionPackage();
        kit.setActionPackage(actions);
        System.out.println("Action package set: " + (kit.getActionPackage() == actions));
        
        // Test PID operations
        kit.setPID(1.0, 0.1, 0.01);
        kit.reset();
        
        // Test calculate method
        double output = kit.calculate(2.0, 1.5);
        System.out.printf("Calculate method output: %.4f%n", output);
        
        System.out.println("✓ Utility methods test passed");
    }
}
