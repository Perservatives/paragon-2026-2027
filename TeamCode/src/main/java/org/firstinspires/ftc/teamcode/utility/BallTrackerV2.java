package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

/**
 * Tracks ball positions in the three-slot selector and automates intake indexing.
 *
 * <p>Ball slots store {@code 0} (empty), {@code 1} (purple), or {@code 2} (green).</p>
 */
public class BallTrackerV2 extends util {

    private static final double SELECTOR_MOVE_TIME = 0.25;
    private static final double SEAT_CONFIRM_TIME = 0.12;

    public final ArrayList<Integer> ballpos = new ArrayList<>(3);
    public final ArrayList<Double> intakePositions = new ArrayList<>();
    public final ArrayList<Double> outtakePositions = new ArrayList<>();

    private boolean autoMode = false;
    private final ElapsedTime seatTimer = new ElapsedTime();
    private boolean selectorMoving = false;
    private final ElapsedTime selectorTimer = new ElapsedTime();
    private final ElapsedTime elapsedTime = new ElapsedTime();

    private int intakePosition = 0;
    private int outtakePosition = -1;

    private final Intake intake;
    private final BallColorDetection color;

    public BallTrackerV2(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        color = new BallColorDetection(hardwareMap);
        selector.scaleRange(0, 1.0);

        for (int i = 0; i < 3; i++) {
            intakePositions.add((i * 0.38) % 1);
            outtakePositions.add((0.16 + (i * 0.38)) % 1);
            ballpos.add(0);
        }

        elapsedTime.reset();
    }

    /** Rotates the selector to the first slot containing {@code color}. */
    public void turnToColor(int color) {
        if (ballpos.contains(color)) {
            outtakePosition = ballpos.indexOf(color);
            selector.setPosition(outtakePositions.get(outtakePosition));
        }
    }

    /** Returns true when all three slots contain a ball. */
    public boolean isFull() {
        return !ballpos.contains(0);
    }

    /** Returns the index of the first empty slot. */
    public int wherethefuckisthezero() {
        return ballpos.indexOf(0);
    }

    /** Returns the number of occupied slots. */
    public int ballsRemaining() {
        int count = 0;
        for (int ball : ballpos) {
            if (ball != 0) {
                count++;
            }
        }
        return count;
    }

    public void setAutoMode(boolean enabled) {
        autoMode = enabled;
    }

    /** Automatically indexes balls into slots while auto mode is enabled. */
    public void updateAutoIntake() {
        if (selectorMoving && selectorTimer.seconds() > SELECTOR_MOVE_TIME) {
            selectorMoving = false;
        }

        if (!autoMode) {
            return;
        }

        if (selectorMoving) {
            seatTimer.reset();
            return;
        }

        if (isFull()) {
            autoMode = false;
            return;
        }

        int detectedColor = color.getColor(2);
        if (detectedColor != 0) {
            if (seatTimer.seconds() > SEAT_CONFIRM_TIME) {
                intakeElement(intakePosition, detectedColor);
                turnToNextIntake();
                seatTimer.reset();
            }
        } else {
            seatTimer.reset();
        }
    }

    public int getOuttakePosition() {
        return outtakePosition;
    }

    public void turnToStartOuttake() {
        outtakePosition = 0;
        selector.setPosition(0.16);
    }

    public void turnToStartIntake() {
        intakePosition = 0;
        selector.setPosition(0);
    }

    /** Advances the selector to the next intake slot and rotates tracked ball positions. */
    public void turnToNextIntake() {
        selectorMoving = true;
        selectorTimer.reset();

        intakePosition = (intakePosition + 1) % 3;
        selector.setPosition(intakePositions.get(intakePosition));

        int placeholder = ballpos.get(2);
        ballpos.set(2, ballpos.get(1));
        ballpos.set(1, ballpos.get(0));
        ballpos.set(0, placeholder);
    }

    /** Advances the selector to the next outtake slot. */
    public void turnToNextOuttake() {
        selectorMoving = true;
        selectorTimer.reset();

        outtakePosition = (outtakePosition + 1) % 3;
        selector.setPosition(outtakePositions.get(outtakePosition));
    }

    public void intakeElement(int position, int element) {
        ballpos.set(position, element);
    }

    public void outtakeElementPos(int position) {
        ballpos.set(position, 0);
    }

    /** Clears the first slot containing {@code element}. */
    public void outtakeElement(int element) {
        ballpos.set(ballpos.indexOf(element), 0);
    }

    /**
     * Collects balls using the legacy detection flow.
     *
     * @deprecated Prefer {@link #getBalls(LinearOpMode, AutoFunctions, int, double)}.
     */
    public void getBalls(LinearOpMode opMode, AutoFunctions auto, int number) {
        final double timeoutSec = 2.5;
        turnToStartIntake();
        int collected = 0;
        boolean found = false;

        tele.addData("IM HERE", "yay");
        tele.update();

        intake.setFast();
        intake.startIntake(false);

        double startTime = elapsedTime.time(TimeUnit.MILLISECONDS);
        while (collected <= number
                && opMode.opModeIsActive()
                && (elapsedTime.time(TimeUnit.MILLISECONDS) - startTime) < (1000 * timeoutSec)) {
            boolean ballDetected = color.ballExists(1);
            if (ballDetected) {
                if (!found) {
                    turnToNextIntake();
                    collected++;
                    found = true;
                }
            }
            if (!ballDetected
                    && found
                    && intakePositions.get(intakePosition) - 0.03 < selector.getPosition()
                    && intakePositions.get(intakePosition) + 0.03 > selector.getPosition()) {
                found = false;
            }

            tele.addData("Remaining in millis:", elapsedTime.time(TimeUnit.MILLISECONDS) - startTime);
            tele.addData("found?", found);
            tele.update();
        }

        auto.stopMotors();
    }

    /**
     * Collects up to {@code number} balls within the given timeout.
     *
     * @param timeoutSec collection timeout in seconds
     */
    public void getBalls(LinearOpMode opMode, AutoFunctions auto, int number, double timeoutSec) {
        turnToStartIntake();
        int collected = 0;
        boolean found = false;

        tele.addData("IM HERE", "yay");
        tele.update();

        intake.setFast();
        intake.startIntake(false);

        double startTime = elapsedTime.time(TimeUnit.MILLISECONDS);
        while (collected < number
                && opMode.opModeIsActive()
                && (elapsedTime.time(TimeUnit.MILLISECONDS) - startTime) < (1000 * timeoutSec)) {
            boolean ballDetected = color.ballExists(2);
            if (ballDetected) {
                if (collected != 2) {
                    turnToNextIntake();
                }
                opMode.sleep(350);
                collected++;
            }

            tele.addData("Remaining in millis:", elapsedTime.time(TimeUnit.MILLISECONDS) - startTime);
            tele.addData("found?", found);
            tele.update();
        }

        opMode.sleep(100);
        auto.stopMotors();
        intake.stopMotor();
    }
}
