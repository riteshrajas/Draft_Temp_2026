package frc.robot.utils.advancedcontrollers;

public class demo {
    Polynomial polynomial = new Polynomial(0.5, 1, 0); // Represents y = 0.5x^2 + x
    CurveKit curveKit = new CurveKit(0.1, 0.01, 0.001, polynomial, 1.0); // PID with polynomial and max output
    Action.ActionPackage actionPackage = new Action.ActionPackage(
        Action.set(value -> System.out.println("Output: " + value), 0.0)
    );

    public demo() {
        curveKit.setActionPackage(actionPackage);
    }

    public void runDemo(double input) {
        curveKit.run(input);
        // This will print the output based on the polynomial and PID calculations
    }

    
}