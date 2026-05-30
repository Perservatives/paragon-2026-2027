package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Controls the intake motor at slow or fast speed in either direction.
 */
public class Intake extends util {

    private static final double SLOW_POWER = 0.2;
    private static final double FAST_POWER = 0.8;

    private boolean isFast = true;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
    }

    /** Use fast intake speed for the next {@link #startIntake(boolean)} call. */
    public void setFast() {
        isFast = true;
    }

    /** Use slow intake speed for the next {@link #startIntake(boolean)} call. */
    public void setSlow() {
        isFast = false;
    }

    /**
     * Runs the intake motor at the currently selected speed.
     *
     * @param forward true for forward rotation, false for reverse
     */
    public void startIntake(boolean forward) {
        double power = isFast ? FAST_POWER : SLOW_POWER;
        intakeMotor.setPower(forward ? power : -power);
    }

    /** Stops the intake motor. */
    public void stopMotor() {
        intakeMotor.setPower(0);
    }
}
