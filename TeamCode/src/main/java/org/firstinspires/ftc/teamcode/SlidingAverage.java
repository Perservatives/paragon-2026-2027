package org.firstinspires.ftc.teamcode;

public class SlidingAverage {
    private final double[] window;
    private int index = 0;
    private int count = 0;
    private double sum = 0;

    public SlidingAverage(int windowSize) {
        window = new double[windowSize];
    }

    public double update(double newValue) {
        sum -= window[index];
        window[index] = newValue;
        sum += newValue;

        index = (index + 1) % window.length;

        if (count < window.length) {
            count++;
        }

        return sum / count;
    }

    public void reset() {
        index = 0;
        count = 0;
        sum = 0;
        for (int i = 0; i < window.length; i++) {
            window[i] = 0;
        }
    }
}

