package frc.robot.utils.sensors;

import java.util.ArrayList;
import java.util.List;

public class SensorFilter {
    
    public static class MovingAverage {
        private final List<Double> values = new ArrayList<>();
        private final int windowSize;
        private double sum = 0;

        public MovingAverage(int windowSize) {
            this.windowSize = windowSize;
        }

        public double calculate(double value) {
            values.add(value);
            sum += value;

            if (values.size() > windowSize) {
                sum -= values.remove(0);
            }

            return sum / values.size();
        }

        public void reset() {
            values.clear();
            sum = 0;
        }
    }

    public static class ExponentialMovingAverage {
        private final double alpha;
        private Double lastValue = null;

        public ExponentialMovingAverage(double alpha) {
            this.alpha = alpha;
        }

        public double calculate(double value) {
            if (lastValue == null) {
                lastValue = value;
                return value;
            }

            lastValue = alpha * value + (1 - alpha) * lastValue;
            return lastValue;
        }

        public void reset() {
            lastValue = null;
        }
    }

    public static class MedianFilter {
        private final List<Double> values = new ArrayList<>();
        private final int windowSize;

        public MedianFilter(int windowSize) {
            this.windowSize = windowSize;
        }

        public double calculate(double value) {
            values.add(value);
            
            if (values.size() > windowSize) {
                values.remove(0);
            }

            List<Double> sortedValues = new ArrayList<>(values);
            sortedValues.sort(Double::compareTo);
            
            int size = sortedValues.size();
            if (size % 2 == 0) {
                return (sortedValues.get(size / 2 - 1) + sortedValues.get(size / 2)) / 2.0;
            } else {
                return sortedValues.get(size / 2);
            }
        }

        public void reset() {
            values.clear();
        }
    }
}
