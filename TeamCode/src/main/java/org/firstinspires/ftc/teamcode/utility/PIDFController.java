package org.firstinspires.ftc.teamcode.utility;

import androidx.annotation.Nullable;

/**
 * PID controller with optional feedforward components for velocity or position control.
 */
public final class PIDFController {

    public static final class PIDCoefficients {
        public double kP, kI, kD;
    }

    public interface FeedforwardFun {
        double compute(double position, @Nullable Double velocity);
    }

    private final PIDCoefficients pid;
    private final double kV, kA, kStatic;
    private final FeedforwardFun kF;

    private double errorSum;
    private long lastUpdateTs;

    private boolean inputBounded;
    private double minInput, maxInput;

    private boolean outputBounded;
    private double minOutput, maxOutput;

    /** Target position (controller setpoint). */
    public double targetPosition;

    /** Target velocity. */
    public double targetVelocity;

    /** Target acceleration. */
    public double targetAcceleration;

    /** Error computed in the last call to {@link #update(long, double, Double)}. */
    public double lastError;

    public PIDFController(
            PIDCoefficients pid,
            double kV,
            double kA,
            double kStatic,
            FeedforwardFun kF
    ) {
        this.pid = pid;
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
        this.kF = kF;
    }

    public PIDFController(PIDCoefficients pid, double kV, double kA, double kStatic) {
        this(pid, kV, kA, kStatic, (x, v) -> 0);
    }

    public PIDFController(PIDCoefficients pid, FeedforwardFun kF) {
        this(pid, 0, 0, 0, kF);
    }

    public PIDFController(PIDCoefficients pid) {
        this(pid, 0, 0, 0);
    }

    /**
     * Sets bounds on controller input. Min and max are treated as the same point when
     * computing wrapped position error.
     */
    public void setInputBounds(double min, double max) {
        if (min < max) {
            inputBounded = true;
            minInput = min;
            maxInput = max;
        }
    }

    /** Sets bounds on controller output. */
    public void setOutputBounds(double min, double max) {
        if (min < max) {
            outputBounded = true;
            minOutput = min;
            maxOutput = max;
        }
    }

    private double getPositionError(double measuredPosition) {
        double error = targetPosition - measuredPosition;
        if (inputBounded) {
            final double inputRange = maxInput - minInput;
            while (Math.abs(error) > inputRange / 2.0) {
                error -= Math.copySign(inputRange, error);
            }
        }
        return error;
    }

    /**
     * Runs a single controller iteration.
     *
     * @param timestamp measurement timestamp from {@link System#nanoTime()}
     * @param measuredPosition measured position feedback
     * @param measuredVelocity measured velocity feedback, or null for position mode
     */
    public double update(long timestamp, double measuredPosition, @Nullable Double measuredVelocity) {
        if (lastUpdateTs == 0) {
            lastUpdateTs = timestamp;
            return 0;
        }

        final double dt = (timestamp - lastUpdateTs) / 1e9;
        lastUpdateTs = timestamp;

        double error;
        double velError;

        if (measuredVelocity != null) {
            error = targetVelocity - measuredVelocity;
            velError = (error - lastError) / dt;
        } else {
            error = getPositionError(measuredPosition);
            velError = (error - lastError) / dt;
        }

        return pid.kP * error;
    }

    public double update(long timestamp, double measuredPosition) {
        return update(timestamp, measuredPosition, null);
    }

    public double update(double measuredPosition) {
        return update(System.nanoTime(), measuredPosition, null);
    }

    /** Resets integral state and timing. */
    public void reset() {
        errorSum = 0;
        lastError = 0;
        lastUpdateTs = 0;
    }
}
