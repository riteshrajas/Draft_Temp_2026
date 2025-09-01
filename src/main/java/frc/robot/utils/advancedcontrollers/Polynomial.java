package frc.robot.utils.advancedcontrollers;


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

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        for (int i = coefficients.length - 1; i >= 0; i--) {
            if (i < coefficients.length - 1) {
                sb.append(" + ");
            }
            sb.append(coefficients[i]);
            if (i > 0) {
                sb.append("x");
                if (i > 1) {
                    sb.append("^").append(i);
                }
            }
        }
        return sb.toString();
    }

    
}