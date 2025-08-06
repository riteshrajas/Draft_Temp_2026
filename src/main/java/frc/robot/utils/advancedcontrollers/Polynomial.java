package frc.robot.utils.advancedcontrollers;

import java.util.Arrays;

public class Polynomial {

    private final double[] coefficients;

    public Polynomial(double... coefficients) {
        this.coefficients = coefficients.clone();
    }

    public double evaluate(double x) {
        double result = 0;
        double power = 1;
        for (int i = 0; i < coefficients.length; i++) {
            result += coefficients[i] * power;
            power *= x;
        }
        return result;
    }

    public int degree() {
        return coefficients.length - 1;
    }

    public Polynomial derivative() {
        if (coefficients.length <= 1) {
            return new Polynomial(0);
        }
        double[] derivativeCoeffs = new double[coefficients.length - 1];
        for (int i = 0; i < coefficients.length; i++) {
            derivativeCoeffs[i - 1] = coefficients[i] * i;
        }
        return new Polynomial(derivativeCoeffs);
    }

    public Polynomial trim() {
        int trueDegree = coefficients.length - 1;
        while (trueDegree > 0 && coefficients[trueDegree] == 0) {
            trueDegree--;
        }
        if (trueDegree == coefficients.length - 1) {
            return this; // Already trimmed
        }
        double[] trimmedCoeffs = new double[trueDegree + 1];
        System.arraycopy(coefficients, 0, trimmedCoeffs, 0, trueDegree + 1);
        return new Polynomial(trimmedCoeffs);
    }

    
}