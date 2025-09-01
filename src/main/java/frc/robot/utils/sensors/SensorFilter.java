package frc.robot.utils.sensors;

import java.util.ArrayList;
import java.util.List;

/**
 * Collection of lightweight, stateful sensor filters useful for robot sensor processing.
 *
 * <p>This utility class contains three common filters implemented as nested static
 * classes. Each filter is intended to be instantiated per sensor signal (they are
 * stateful) and are <strong>not</strong> thread-safe. Typical usage is to create one
 * filter instance for a sensor channel and call {@code calculate(double)} on each new
 * sample, or call {@code reset()} to clear internal history.</p>
 *
 * Available filters:
 * <ul>
 *   <li>{@link MovingAverage} — sliding (boxcar) average over the most recent N samples.</li>
 *   <li>{@link ExponentialMovingAverage} — single‑pole IIR low‑pass (EMA), constant memory.</li>
 *   <li>{@link MedianFilter} — sliding window median, robust to impulsive outliers.</li>
 * </ul>
 *
 * Contract (per filter instance):
 * <ul>
 *   <li>Inputs: raw sensor sample (double).</li>
 *   <li>Outputs: filtered sample (double).</li>
 *   <li>Error modes: invalid constructor parameters (e.g. non‑positive window size, alpha outside (0,1]) are
 *       not explicitly checked by these lightweight implementations — callers should ensure valid args.</li>
 *   <li>Success criteria: filtered output is returned on each {@code calculate} call and internal state updates accordingly.</li>
 * </ul>
 */
public class SensorFilter {

    /**
     * Sliding (boxcar) moving average over the most recent {@code windowSize} samples.
     * <p>
     * Characteristics:
     * <ul>
     *   <li>Linear phase, introduces latency ~ (N-1)/2 samples.</li>
     *   <li>Memory: stores up to {@code windowSize} values.</li>
     *   <li>Time complexity: amortized O(1) per sample; removal from a {@link java.util.ArrayList}
     *       is O(N) when the list shifts — for large windows consider a circular buffer.</li>
     * </ul>
     * </p>
     */
    public static class MovingAverage {
        private final List<Double> values = new ArrayList<>();
        private final int windowSize;
        private double sum = 0;

        /**
         * Create a moving average filter.
         *
         * @param windowSize number of recent samples to average; must be &gt; 0 (caller responsibility).
         */
        public MovingAverage(int windowSize) {
            this.windowSize = windowSize;
        }

        /**
         * Add a new sample and return the current moving average. Until {@code windowSize}
         * samples have been seen the average is computed over the available samples.
         *
         * @param value new raw sample
         * @return average of last min(sampleCount, windowSize) samples
         */
        public double calculate(double value) {
            values.add(value);
            sum += value;

            if (values.size() > windowSize) {
                // remove oldest value and update running sum
                sum -= values.remove(0);
            }

            return sum / values.size();
        }

        /**
         * Clears stored samples and resets the running sum to zero.
         */
        public void reset() {
            values.clear();
            sum = 0;
        }
    }

    /**
     * Exponential Moving Average (single‑pole IIR) implemented as:
     * <pre>y[n] = alpha * x[n] + (1 - alpha) * y[n-1]</pre>
     *
     * Characteristics:
     * <ul>
     *   <li>Constant memory (stores only last output).</li>
     *   <li>Low latency relative to boxcar; smoothing controlled by {@code alpha}.</li>
     * </ul>
     */
    public static class ExponentialMovingAverage {
        private final double alpha;
        private Double lastValue = null; // null indicates uninitialized

        /**
         * Create an EMA filter.
         *
         * @param alpha smoothing factor in (0, 1]. Larger alpha weights new samples more.
         */
        public ExponentialMovingAverage(double alpha) {
            this.alpha = alpha;
        }

        /**
         * Add a new sample and return the EMA output. The first call initializes the
         * internal state and returns the raw sample.
         *
         * @param value new raw sample
         * @return filtered value
         */
        public double calculate(double value) {
            if (lastValue == null) {
                // initialize on first sample to avoid bias
                lastValue = value;
                return value;
            }

            lastValue = alpha * value + (1 - alpha) * lastValue;
            return lastValue;
        }

        /**
         * Reset the filter so the next sample re-initializes internal state.
         */
        public void reset() {
            lastValue = null;
        }
    }

    /**
     * Sliding-window median filter. Robust to short impulsive spikes / outliers.
     *
     * Note: This naive implementation sorts a copy of the current buffer each call
     * (O(N log N)). For larger windows or high sample rates consider a more
        * sophisticated data structure (e.g. two heaps) to maintain the median in O(log N).
     */
    public static class MedianFilter {
        private final List<Double> values = new ArrayList<>();
        private final int windowSize;

        /**
         * Create a median filter.
         *
         * @param windowSize number of recent samples to consider; must be &gt; 0 (caller responsibility).
         */
        public MedianFilter(int windowSize) {
            this.windowSize = windowSize;
        }

        /**
         * Add a new sample and return the current median of the buffered samples.
         * If the buffer contains an even number of samples the returned value is
         * the average of the two central sorted samples.
         *
         * @param value new raw sample
         * @return median value
         */
        public double calculate(double value) {
            values.add(value);

            if (values.size() > windowSize) {
                // drop oldest
                values.remove(0);
            }

            List<Double> sortedValues = new ArrayList<>(values);
            sortedValues.sort(Double::compareTo);

            int size = sortedValues.size();
            if (size % 2 == 0) {
                // even -> average two middle elements
                return (sortedValues.get(size / 2 - 1) + sortedValues.get(size / 2)) / 2.0;
            } else {
                return sortedValues.get(size / 2);
            }
        }

        /**
         * Clears the stored samples.
         */
        public void reset() {
            values.clear();
        }
    }
}
