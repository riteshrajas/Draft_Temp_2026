package frc.robot.utils.motion;

import edu.wpi.first.wpilibj.Timer;

public class SlewRateLimiter {
    private final double positiveRateLimit;
    private final double negativeRateLimit;
    private double prevVal = 0;
    private double prevTime = Timer.getFPGATimestamp();

    public SlewRateLimiter(double rateLimit) {
        this(rateLimit, -rateLimit);
    }

    public SlewRateLimiter(double positiveRateLimit, double negativeRateLimit) {
        this.positiveRateLimit = positiveRateLimit;
        this.negativeRateLimit = negativeRateLimit;
    }

    public double calculate(double input) {
        double currentTime = Timer.getFPGATimestamp();
        double elapsedTime = currentTime - prevTime;
        
        if (elapsedTime <= 0) {
            return prevVal;
        }
        
        double maxChange = (input > prevVal) ? 
            positiveRateLimit * elapsedTime : 
            negativeRateLimit * elapsedTime;
        
        double change = input - prevVal;
        
        if (Math.abs(change) > Math.abs(maxChange)) {
            change = maxChange;
        }
        
        prevVal += change;
        prevTime = currentTime;
        
        return prevVal;
    }

    public void reset(double value) {
        prevVal = value;
        prevTime = Timer.getFPGATimestamp();
    }

    public double getLastValue() {
        return prevVal;
    }
}
