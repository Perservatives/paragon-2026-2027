package org.firstinspires.ftc.teamcode.utility;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AxonServo;
import org.firstinspires.ftc.teamcode.HardwareInit;
import org.firstinspires.ftc.teamcode.LimeLight;

import java.util.ArrayList;

/**
 * High-level autonomous routines for drivetrain movement, turning, and shooting.
 *
 * <p>Extends {@link HardwareInit} and composes intake, ball tracking, color detection,
 * flywheel PID, and Limelight helpers used by auto op modes.</p>
 */
public class AutoFunctions extends HardwareInit {

    private static final double MAX_RPM = 5600.0;

    // --- PID coefficients (flywheel) ---
    public static double kP = 0.0005;
    public static double kI = 0.0;
    public static double kD = 0.00;

    // --- Drivetrain motor stats ---
    public static double driveCPR = 384.5;
    public static double driveRPM = 435;

    public static double startVoltage;

    public final Intake intake;
    public final BallTrackerV2 ballTrackerV2;
    public final BallColorDetection ballColorDetection;
    public final AxonServo axonServo;
    public final PIDFController pidf;
    public final LimeLight ll;

    public AutoFunctions(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);

        intake = new Intake(hardwareMap, tele);
        ballTrackerV2 = new BallTrackerV2(hardwareMap, tele);
        ballColorDetection = new BallColorDetection(hardwareMap);
        axonServo = new AxonServo();
        ll = new LimeLight(limelight, telemetry);

        PIDFController.PIDCoefficients coeff = new PIDFController.PIDCoefficients();
        coeff.kP = kP;
        coeff.kI = kI;
        coeff.kD = kD;

        double kV = 1.0 / MAX_RPM;
        pidf = new PIDFController(coeff, kV, 0, 0);

        startVoltage = voltageSensor.getVoltage();
    }

    // -------------------------------------------------------------------------
    // Drivetrain helpers
    // -------------------------------------------------------------------------

    /**
     * Converts a positional error into motor power using proportional control.
     *
     * @param maxPower upper bound on output power
     * @param error distance or angle error
     * @param kp proportional gain
     */
    public double adjustError(double maxPower, double error, double kp) {
        double power = Math.abs(error) * kp;
        power = Math.min(maxPower, power);

        double minPower = 0.1;
        if (0.005 < Math.abs(power) && Math.abs(power) < minPower) {
            power = minPower;
        }

        return Math.signum(error) * power;
    }

    /** Stops all drivetrain motors and waits briefly for momentum to settle. */
    public void stopMotors() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        fl.setVelocity(0);
        fr.setVelocity(0);
        bl.setVelocity(0);
        br.setVelocity(0);

        try {
            Thread.sleep(150);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /** Sets equal power on all drivetrain motors. */
    public void startMotors(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }

    /** Resets odometry and IMU heading. */
    public void resetOdo() {
        odo.resetPosAndIMU();
        imu.resetYaw();
    }

    // -------------------------------------------------------------------------
    // Field-relative movement
    // -------------------------------------------------------------------------

    /**
     * Drives to a field position using power-based holonomic control.
     *
     * <p>Coordinates are in millimeters. Stops when within 7 mm of the target.</p>
     */
    public void moveTo(@NonNull LinearOpMode opMode, double xdest, double ydest, double maxPower, double kp) {
        while (opMode.opModeIsActive()) {
            odo.update();
            double x = odo.getPosX(DistanceUnit.MM);
            double y = odo.getPosY(DistanceUnit.MM);
            double xError = xdest - x;
            double yError = ydest - y;
            double xPower = adjustError(maxPower, xError, kp);
            double yPower = adjustError(maxPower, yError, kp);

            fl.setPower(xPower - yPower);
            bl.setPower((xPower + yPower) * 0.95);
            fr.setPower(xPower + yPower);
            br.setPower((xPower - yPower) * 0.95);

            tele.addData("X:", Math.round(x));
            tele.addData("Y:", Math.round(y));
            tele.addData("Yaw:", imu.getRobotYawPitchRollAngles().getYaw());
            tele.addData("Pitch:", imu.getRobotYawPitchRollAngles().getPitch());
            tele.addData("Roll:", imu.getRobotYawPitchRollAngles().getRoll());
            tele.update();

            if (Math.abs(Math.hypot(xError, yError)) < 7) {
                tele.addData("The robot x SHOULD be at", xdest);
                tele.addData("The robot y SHOULD be at", ydest);
                tele.addData("Robot actual location x:", x);
                tele.addData("Robot actual location y:", y);
                tele.update();
                break;
            }
        }
        stopMotors();
    }

    /** Moves through a sequence of waypoints without stopping between segments. */
    public void chainMovement(
            @NonNull LinearOpMode opMode,
            ArrayList<Double> xPoints,
            ArrayList<Double> yPoints,
            double maxPower
    ) {
        for (int i = 0; i < xPoints.size(); i++) {
            moveToV2Chained(opMode, xPoints.get(i), yPoints.get(i), maxPower);
        }
    }

    /**
     * Drives to a field position using encoder velocity for improved accuracy.
     *
     * <p>Coordinates are in inches. Stops when within 1.5 inches of the target.</p>
     */
    public void moveToV2(@NonNull LinearOpMode opMode, double xdest, double ydest, double maxVel) {
        moveToV2Internal(opMode, xdest, ydest, maxVel, 0.03, true);
    }

    /**
     * Same as {@link #moveToV2} but does not stop motors at the end, for chained paths.
     *
     * <p>Coordinates are in inches.</p>
     */
    public void moveToV2Chained(@NonNull LinearOpMode opMode, double xdest, double ydest, double maxVel) {
        moveToV2Internal(opMode, xdest, ydest, maxVel, 0.02, false);
    }

    private void moveToV2Internal(
            LinearOpMode opMode,
            double xdest,
            double ydest,
            double maxVel,
            double kp,
            boolean stopAtEnd
    ) {
        odo.update();
        double maxPower = maxVel / driveRPM;

        while (opMode.opModeIsActive()) {
            odo.update();
            double x = odo.getPosX(DistanceUnit.INCH);
            double y = odo.getPosY(DistanceUnit.INCH);
            double xError = xdest - x;
            double yError = ydest - y;
            double currentTotalError = Math.abs(Math.hypot(xError, yError));
            double xPower = adjustError(maxPower, xError, kp);
            double yPower = adjustError(maxPower, yError, kp);

            double xVel = xPower * maxVel * driveCPR / 60;
            double yVel = yPower * maxVel * driveCPR / 60;
            fl.setVelocity(xVel - yVel);
            bl.setVelocity(xVel + yVel);
            fr.setVelocity(xVel + yVel);
            br.setVelocity(xVel - yVel);

            tele.addData("Robot actual location x:", x);
            tele.addData("Robot actual location y:", y);
            tele.update();

            if (currentTotalError < 1.5) {
                tele.addData("The robot x SHOULD be at", xdest);
                tele.addData("The  robot y SHOULD be at", ydest);
                tele.addData("Robot actual location x:", x);
                tele.addData("Robot actual location y:", y);
                tele.update();
                break;
            }
        }

        if (stopAtEnd) {
            stopMotors();
        }
    }

    // -------------------------------------------------------------------------
    // Turning
    // -------------------------------------------------------------------------

    private double normalizeDeg(double angle) {
        while (angle <= -180) {
            angle += 360;
        }
        while (angle > 180) {
            angle -= 360;
        }
        return angle;
    }

    /** Turns the robot by a relative angle in degrees. */
    public void Turn(LinearOpMode opMode, double angle, double maxPower) {
        final double kp = 0.025;
        angle = -angle;

        double startHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double targetHeading = startHeading + angle;

        while (opMode.opModeIsActive()) {
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double remaining = normalizeDeg(targetHeading - currentHeading);
            double power = adjustError(maxPower, remaining, kp);

            fl.setPower(-power);
            bl.setPower(-power);
            fr.setPower(power);
            br.setPower(power);

            if (Math.abs(remaining) < 0.2) {
                break;
            }

            tele.addData("yaw", currentHeading);
            tele.addData("remaining", remaining);
            tele.update();
        }
        stopMotors();
    }

    /** Turns to an absolute heading in degrees. */
    public void TurnTo(LinearOpMode opMode, double angle, double maxPower) {
        final double kp = 0.04;
        angle = normalizeDeg(angle);

        while (opMode.opModeIsActive()) {
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double remaining = normalizeDeg(angle - currentHeading);
            double power = adjustError(maxPower, remaining, kp);

            fl.setPower(-power);
            bl.setPower(-power);
            fr.setPower(power);
            br.setPower(power);

            if (Math.abs(remaining) < 1) {
                break;
            }

            tele.addData("Yaw:", currentHeading);
            tele.update();
        }
        stopMotors();
    }

    /** Turns to an absolute heading using encoder velocity. */
    public void TurnToV2(LinearOpMode opMode, double angle, int maxRPM) {
        final double kp = 0.02;
        angle = normalizeDeg(angle);
        double maxPower = maxRPM / driveRPM;

        while (opMode.opModeIsActive()) {
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double remaining = normalizeDeg(angle - currentHeading);
            double power = adjustError(maxPower, remaining, kp);

            double velocity = power * maxRPM * driveCPR / 60;
            fl.setVelocity(-velocity);
            bl.setVelocity(-velocity);
            fr.setVelocity(velocity);
            br.setVelocity(velocity);

            tele.addData("Velocity", velocity);

            if (Math.abs(remaining) < 0.5) {
                break;
            }

            tele.addData("Yaw:", currentHeading);
            tele.update();
        }
        stopMotors();
    }

    // -------------------------------------------------------------------------
    // Shooter helpers
    // -------------------------------------------------------------------------

    /**
     * Estimates motor RPM from encoder delta over one loop iteration.
     *
     * @param motor encoder-equipped motor
     * @param opMode running op mode for timing
     * @param countsPerRev encoder counts per revolution
     */
    public double calculateRPM(DcMotor motor, OpMode opMode, double countsPerRev) {
        double prevPos = motor.getCurrentPosition();
        double prevTime = opMode.getRuntime();

        double currPos = motor.getCurrentPosition();
        double currTime = opMode.getRuntime();

        double deltaPos = currPos - prevPos;
        double deltaTime = currTime - prevTime;
        if (deltaTime <= 0) {
            deltaTime = 0.001;
        }

        double revolutions = deltaPos / countsPerRev;
        return (revolutions / deltaTime) * 60.0;
    }

    /**
     * Prepares the shooter for firing.
     *
     * @return fixed motor power placeholder used by {@link #shoot}
     */
    public double prepareShoot(LinearOpMode opMode) {
        return 95.3;
    }

    /**
     * Computes the launch angle for a projectile given velocity, distance, and height.
     *
     * @return angle in radians, or {@link Double#NaN} if the shot is impossible
     */
    public double get_shooting_angle(double motorVel, double shootingDistance, double shootingHeight) {
        double g = 9.8;
        double v2 = motorVel * motorVel;
        double inside = v2 * v2 - g * (g * shootingDistance * shootingDistance + 2 * shootingHeight * v2);

        if (inside < 0 || shootingDistance == 0) {
            return Double.NaN;
        }

        return Math.atan((v2 - Math.sqrt(inside)) / (g * shootingDistance));
    }

    /** Returns the turret angle needed to aim at a field target. */
    public double get_turret_angle(double targetX, double targetY) {
        double x = odo.getPosition().getX(DistanceUnit.INCH);
        double y = odo.getPosition().getY(DistanceUnit.INCH);
        double baseRotation = imu.getRobotYawPitchRollAngles().getYaw();

        return Math.atan2(targetY - y, targetX - x) - baseRotation;
    }

    /** Returns distance in inches from the current pose to a starting point. */
    public double get_distance(double startingX, double startingY) {
        double x = odo.getPosition().getX(DistanceUnit.INCH);
        double y = odo.getPosition().getY(DistanceUnit.INCH);
        return Math.sqrt(Math.pow(startingX - x, 2) + Math.pow(startingY - y, 2));
    }

    /** Experimental shooting routine that adjusts hood angle from ballistics. */
    public void shoot_v4(LinearOpMode opMode, double targetX, double targetY) {
        ballTrackerV2.turnToStartIntake();
        stopMotors();
        motor.setPower(0.7);
        get_turret_angle(targetX, targetY);

        for (int i = 0; i < 3; i++) {
            transfer.setPower(-1);
            while (Double.isNaN(get_shooting_angle(motor.getVelocity(), get_distance(targetX, targetY), 123123))) {
                motor.setPower(1);
            }
            ballTrackerV2.turnToStartOuttake();
        }
        motor.setPower(0.7);
    }

    /** Limelight turret calibration placeholder. */
    public void calibrate(LinearOpMode opMode) {
    }

    /**
     * Fires one ball using Limelight alignment and a hood servo burst.
     *
     * @param offset desired tx offset from the AprilTag
     */
    public void shootV3(LinearOpMode opMode, double shootVel, double offset) {
        stopMotors();
        ll.loop();

        while (opMode.opModeIsActive()
                && (motor.getVelocity() < shootVel
                || ll.txAvgVal > offset + 0.3
                || ll.txAvgVal < offset - 0.3)) {
            motor.setVelocity(shootVel + 5);
            ll.loop();

            if (ll.txAvgVal > offset + 0.3) {
                roto.setPower(-0.09);
            } else if (ll.txAvgVal < offset - 0.3) {
                roto.setPower(0.09);
            } else {
                roto.setPower(0);
            }

            tele.addData("ticks", motor.getVelocity());
            tele.addData("tx", ll.txAvgVal);
            tele.update();
        }

        servoTest.setPosition(0.085);
        ll.loop();
        roto.setPower(0);
        ballTrackerV2.selector.setPosition(0);
        transfer.setPower(-1);
        ballTrackerV2.selector.setPosition(1);
        opMode.sleep(1000);
        transfer.setPower(0);
        ballTrackerV2.turnToStartOuttake();
    }

    /**
     * Fires three balls at long range with higher flywheel velocity boost.
     *
     * @param uptime delay between shots in milliseconds
     */
    public void shootV2far(LinearOpMode opMode, double shootVel, long uptime, double offset) {
        ballTrackerV2.turnToStartIntake();
        stopMotors();
        ll.loop();

        for (int i = 0; i < 3; i++) {
            while (opMode.opModeIsActive()
                    && (motor.getVelocity() < shootVel
                    || ll.txAvgVal > offset + 0.3
                    || ll.txAvgVal < offset - 0.3)) {
                motor.setVelocity(shootVel + 150);
                ll.loop();

                if (ll.txAvgVal > offset + 0.3) {
                    roto.setPower(-0.09);
                } else if (ll.txAvgVal < offset - 0.3) {
                    roto.setPower(0.09);
                } else {
                    roto.setPower(0);
                }

                tele.addData("ticks", motor.getVelocity());
                tele.addData("tx", ll.txAvgVal);
                tele.update();
            }

            ll.loop();
            roto.setPower(0);
            transfer.setPower(-1);
            opMode.sleep(100);
            tele.addData("shoot ticks", motor.getVelocity());
            tele.addData("shoot tx", ll.txAvgVal);
            tele.addData("current cycle", i);
            tele.update();
            ballTrackerV2.turnToNextOuttake();
            opMode.sleep(uptime);
        }

        opMode.sleep(300);
        transfer.setPower(0);
        ballTrackerV2.turnToStartIntake();
    }

    /**
     * Fires three balls with Limelight alignment.
     *
     * @param uptime delay between shots in milliseconds
     */
    public void shootV2(LinearOpMode opMode, double shootVel, long uptime, double offset) {
        ballTrackerV2.turnToStartIntake();
        stopMotors();
        ll.loop();

        for (int i = 0; i < 3; i++) {
            while (opMode.opModeIsActive()
                    && (motor.getVelocity() < shootVel
                    || ll.txAvgVal > offset + 0.3
                    || ll.txAvgVal < offset - 0.3)) {
                motor.setVelocity(shootVel + 5);
                ll.loop();

                if (ll.txAvgVal > offset + 0.3) {
                    roto.setPower(-0.10);
                } else if (ll.txAvgVal < offset - 0.3) {
                    roto.setPower(0.10);
                } else {
                    roto.setPower(0);
                }

                tele.addData("ticks", motor.getVelocity());
                tele.addData("tx", ll.txAvgVal);
                tele.update();
            }

            ll.loop();
            roto.setPower(0);
            transfer.setPower(-1);
            opMode.sleep(100);
            tele.addData("shoot ticks", motor.getVelocity());
            tele.addData("shoot tx", ll.txAvgVal);
            tele.addData("current cycle", i);
            tele.update();
            ballTrackerV2.turnToNextOuttake();
            opMode.sleep(uptime);
        }

        opMode.sleep(300);
        transfer.setPower(0);
        ballTrackerV2.turnToStartIntake();
    }

    /**
     * Shoots balls in a specified color order, or any available balls if order is empty.
     *
     * @param order color sequence ({@code 1} = purple, {@code 2} = green), or empty for any
     */
    public void shoot(LinearOpMode opMode, int[] order) {
        if (order.length != 0) {
            for (int color : order) {
                motor.setPower(prepareShoot(opMode));
                ballTrackerV2.turnToColor(color);
                transfer.setPower(-1);
                ballTrackerV2.outtakeElement(color);
                try {
                    Thread.sleep(400);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
                transfer.setPower(0);
            }
            transfer.setPower(0);
        } else {
            for (int i = 0; i < 3; i++) {
                motor.setPower(prepareShoot(opMode));
                transfer.setPower(-1);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
                transfer.setPower(0);
            }
        }
        motor.setPower(0.7);
    }

    /** Shoots three balls at long range using fixed high power. */
    public void shootFar(LinearOpMode opMode, int rpmIncrease) {
        for (int i = 0; i < 3; i++) {
            calibrate(opMode);
            motor.setPower(0.98);
            transfer.setPower(-1);
            opMode.sleep(800);
            ballTrackerV2.turnToNextOuttake();
            opMode.sleep(600);
            transfer.setPower(0);
        }
    }
}
