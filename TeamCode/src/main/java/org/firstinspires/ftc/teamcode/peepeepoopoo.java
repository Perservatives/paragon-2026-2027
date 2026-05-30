package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utility.BallColorDetection;
import org.firstinspires.ftc.teamcode.utility.BallTrackerV2;
import org.firstinspires.ftc.teamcode.utility.PIDFController;

/**
 * Primary TeleOp for driver control, auto-aim shooting, and ball indexing.
 *
 * <p>Gamepad 1 drives the robot. Gamepad 2 controls intake, transfer, selector,
 * flywheel, hood, and turret alignment via Limelight.</p>
 *
 * <p>Limelight polling and turret/hood auto-aim run on a background thread so aiming
 * stays responsive while the main loop handles driving and ball indexing.</p>
 */
@TeleOp(name = "peepeepoopoo", group = "Linear OpMode")
public class peepeepoopoo extends LinearOpMode {

    private static final double TARGET_RPM = (312.0 / 312.0) * 5600.0 * (4.0 / 3.0);
    private static final double CPR = 28.0;
    private static final double MAX_RPM = 6440.0;

    private static final double PAUSE_DURATION = 0.7;
    private static final double DELAY_SECONDS = 0.75;
    private static final long VISION_LOOP_MS = 10;

    public static double kP = 0.0005;
    public static double kI = 0.0;
    public static double kD = 0.00;

    private final ElapsedTime delayTimer = new ElapsedTime();
    private final ElapsedTime pauseTimer = new ElapsedTime();
    private final ElapsedTime shootTimer = new ElapsedTime();
    private final ElapsedTime rotoAdjusting = new ElapsedTime();

    private volatile boolean opModeRunning = false;
    private volatile boolean gameState = true;
    private volatile boolean shootingAll = false;
    private volatile boolean autoIntaking = false;

    private boolean pauseActive = false;
    private boolean rotoAdjust = false;
    private boolean aPressed = false;
    private boolean waiting = false;
    private boolean finishingLastShot = false;

    private int ballsShot = 0;
    private int counter = 0;

    private volatile double lastFeedVelocity = 1600.0;
    private volatile double lastFeedAngle = 0.0;
    private volatile double lastDistanceInches = Double.NaN;

    private double speed = 1457.6143;
    private double fuck4Balls = 0.7;

    private double flywheelPrevPos;
    private double flywheelPrevTime;
    private double flywheelPrevRpm;

    private TouchSensor test_magnetic;
    private Servo servoTest;
    private Servo selector;
    private DcMotor transfer;
    private DcMotor IN;
    private DcMotor roto;
    private DcMotorEx motor;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private GoBildaPinpointDriver odo;
    private Limelight3A limelight;
    private LimeLight ll;
    private PIDFController pidf;
    private AxonServo axonServo;

    private Thread visionThread;

    @Override
    public void runOpMode() {
        new BallColorDetection(hardwareMap);
        BallTrackerV2 bt = new BallTrackerV2(hardwareMap, telemetry);
        hardwareMap.get(AnalogInput.class, "selectorInfo");

        motor = hardwareMap.get(DcMotorEx.class, "motorTest");
        IN = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        roto = hardwareMap.get(DcMotor.class, "turretMotor");
        roto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        test_magnetic = hardwareMap.get(TouchSensor.class, "testDis");
        servoTest = hardwareMap.get(Servo.class, "servoTest");
        selector = hardwareMap.get(Servo.class, "selector");
        axonServo = new AxonServo();

        PIDFController.PIDCoefficients coeff = new PIDFController.PIDCoefficients();
        coeff.kP = kP;
        coeff.kI = kI;
        coeff.kD = kD;
        pidf = new PIDFController(coeff, 1.0 / MAX_RPM, 0, 0);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        ll = new LimeLight(limelight, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Robot Ready. Press Play.");
        waitForStart();

        flywheelPrevPos = motor.getCurrentPosition();
        flywheelPrevTime = getRuntime();
        flywheelPrevRpm = 0;

        initDrivetrain();
        initOdometry();

        Pose2D startPose = odo != null ? odo.getPosition() : null;

        opModeRunning = true;
        visionThread = new Thread(this::visionLoop, "peepeepoopoo-vision");
        visionThread.start();

        while (opModeIsActive()) {
            driveRobot();

            if (odo != null) {
                updateOdometryTelemetry(startPose);
            }

            updateFlywheelPid();

            updateHoodManual();

            handleSelectorIntake(bt);
            handleSelectorOuttake(bt);
            handleGameStateToggle();

            if (gameState) {
                runAutoAimMode(bt);
            } else {
                runManualShootMode(bt);
            }

            updateAutoIntake(bt);
            updateIdleTransfer();

            handleIntakeToggle();
            handleHoodPresets();
            handleFlywheelTrim();

            telemetry.addData("tx", ll.txAvgVal);
            telemetry.addData("ty", ll.tyAvgVal);
            telemetry.addData("distanceFromGoal", lastDistanceInches);
            telemetry.addData("ballsShot", ballsShot);
            telemetry.addData("isFull", bt.isFull());
            telemetry.addData("zeros???", bt.wherethefuckisthezero());
            telemetry.addData("Speed", motor.getVelocity());
            telemetry.addData("Servo Pos", servoTest.getPosition());
            telemetry.addData("Selector", selector.getPosition());
            telemetry.update();
        }

        opModeRunning = false;
        if (visionThread != null) {
            try {
                visionThread.join(500);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        motor.setPower(0);
        roto.setPower(0);
    }

    /**
     * Background loop: polls Limelight and drives turret / auto hood+flywheel
     * independently of drivetrain and indexing on the main thread.
     */
    private void visionLoop() {
        while (opModeRunning && opModeIsActive()) {
            ll.updateVision();

            if (!shootingAll) {
                updateTurretVision();
                if (gameState) {
                    updateAutoShooterVision();
                }
            }

            try {
                Thread.sleep(VISION_LOOP_MS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
    }

    /** Turret auto-align and manual fallback, runs on the vision thread. */
    private void updateTurretVision() {
        double K = test_magnetic.isPressed() ? 20 : 3;

        if (ll.txAvgVal > 4.5) {
            roto.setPower(-0.50 / K);
        } else if (ll.txAvgVal < -4.5) {
            roto.setPower(0.50 / K);
        } else {
            roto.setPower(gamepad2.left_stick_x / 5.0);
        }
    }

    /** Distance-based hood and flywheel tuning from Limelight ty, runs on the vision thread. */
    private void updateAutoShooterVision() {
        if (Double.isNaN(ll.tyAvgVal)) {
            motor.setVelocity(lastFeedVelocity);
            return;
        }

        double targetOffsetAngleVertical = ll.tyAvgVal;
        double limelightLensHeightInches = 9.8;
        double limelightMountAngleDegrees = 10.0;
        double goalHeightInches = 29.5;
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngleVertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
        double distanceFromLimelightToGoalInches =
                (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        lastDistanceInches = distanceFromLimelightToGoalInches;
        lastFeedVelocity = (0.000774246 * Math.pow(distanceFromLimelightToGoalInches, 4)
                - 0.209337 * Math.pow(distanceFromLimelightToGoalInches, 3)
                + 20.66155 * Math.pow(distanceFromLimelightToGoalInches, 2)
                - 878.94092 * distanceFromLimelightToGoalInches
                + 15341.0551);
        lastFeedAngle = ((3.38833 * 0.00000001) * Math.pow(distanceFromLimelightToGoalInches, 4)
                - 0.0000118103 * Math.pow(distanceFromLimelightToGoalInches, 3)
                + 0.00143564 * Math.pow(distanceFromLimelightToGoalInches, 2)
                - 0.073255 * distanceFromLimelightToGoalInches
                + 1.43108);

        motor.setVelocity(lastFeedVelocity);
        servoTest.setPosition(lastFeedAngle);
    }

    private void initDrivetrain() {
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    private void initOdometry() {
        try {
            odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
            odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odo.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            );
            odo.resetPosAndIMU();

            telemetry.addData("Status", "Initialized");
            telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
            telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
            telemetry.addData("Device Version", odo.getDeviceVersion());
            telemetry.addData("Heading Scalar", odo.getYawScalar());
        } catch (Exception e) {
            odo = null;
            telemetry.addData("Odometry", "Not Found");
        }
    }

    private void driveRobot() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double frontLeftPower = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backLeftPower = drive - strafe + turn;
        double backRightPower = drive + strafe - turn;

        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        telemetry.addData("Drive", "%.2f", drive);
        telemetry.addData("Strafe", "%.2f", strafe);
        telemetry.addData("Turn", "%.2f", turn);
        telemetry.addData("FL Power", "%.2f", frontLeftPower);
        telemetry.addData("FR Power", "%.2f", frontRightPower);
        telemetry.addData("BL Power", "%.2f", backLeftPower);
        telemetry.addData("BR Power", "%.2f", backRightPower);
    }

    private void updateOdometryTelemetry(Pose2D startPose) {
        odo.update();
        Pose2D currentPose = odo.getPosition();
        double dx = currentPose.getX(DistanceUnit.MM) - startPose.getX(DistanceUnit.MM);
        double dy = currentPose.getY(DistanceUnit.MM) - startPose.getY(DistanceUnit.MM);
        double distanceFromStartIn = Math.hypot(dx, dy) / 25.4;
        telemetry.addData("DistanceInches", distanceFromStartIn);
    }

    private void updateFlywheelPid() {
        double currPos = motor.getCurrentPosition();
        double currTime = getRuntime();

        double deltaPos = currPos - flywheelPrevPos;
        double deltaTime = currTime - flywheelPrevTime;
        if (deltaTime <= 0) {
            deltaTime = 0.001;
        }

        double revolutionsFly = deltaPos / CPR;
        double rpm = (revolutionsFly / deltaTime) * 60.0;
        if (flywheelPrevRpm != 0) {
            rpm = (rpm + flywheelPrevRpm) / 2.0;
        }
        flywheelPrevRpm = rpm;

        pidf.targetVelocity = TARGET_RPM;
        double output = pidf.update(System.nanoTime(), motor.getCurrentPosition(), rpm);
        output = Math.max(-1.0, Math.min(1.0, output));

        flywheelPrevPos = currPos;
        flywheelPrevTime = currTime;
    }

    private void updateHoodManual() {
        telemetry.addData("speed", speed);
        telemetry.addData("angle", servoTest.getPosition());

        if (test_magnetic.isPressed()) {
            telemetry.addData("True", "true!");
        }

        if (ll.tyAvgVal > -12.5) {
            if (gamepad2.left_stick_y >= 0.5) {
                telemetry.addData("Testing", "REAL");
                servoTest.setPosition(servoTest.getPosition() - 0.005);
            } else if (gamepad2.left_stick_y <= -0.5) {
                servoTest.setPosition(servoTest.getPosition() + 0.005);
            }
        }
    }

    private void handleSelectorIntake(BallTrackerV2 bt) {
        if (gamepad2.aWasPressed() && !aPressed) {
            aPressed = true;
            waiting = true;
            delayTimer.reset();
            bt.turnToNextIntake();
        }

        if (waiting && delayTimer.seconds() >= DELAY_SECONDS) {
            transfer.setPower(0);
            aPressed = false;
            waiting = false;
        }
    }

    private void handleSelectorOuttake(BallTrackerV2 bt) {
        if (gamepad2.xWasPressed() && !aPressed) {
            aPressed = true;
            waiting = true;
            delayTimer.reset();

            bt.ballpos.remove(1);
            bt.ballpos.add(1, 0);
            bt.turnToNextOuttake();

            pauseTimer.reset();
            pauseActive = true;
            transfer.setPower(0);
        }

        if (pauseActive && pauseTimer.seconds() >= PAUSE_DURATION) {
            transfer.setPower(-1);
            counter--;
            pauseActive = false;
        }

        if (waiting && delayTimer.seconds() >= DELAY_SECONDS) {
            transfer.setPower(0);
            waiting = false;
            aPressed = false;
        }
    }

    private void handleGameStateToggle() {
        if (gamepad2.dpad_right) {
            if (gameState) {
                gameState = false;
            } else {
                gameState = true;
            }
        }
    }

    /** Burst-fire sequence; turret control is handled on the main thread while shooting. */
    private void runAutoAimMode(BallTrackerV2 bt) {
        if (gamepad2.bWasPressed() && !shootingAll && !autoIntaking) {
            shootingAll = true;
            ballsShot = 0;
            shootTimer.reset();
            bt.turnToStartOuttake();
        }

        if (shootingAll) {
            roto.setPower(0);
            rotoAdjust = true;
            rotoAdjusting.reset();
            transfer.setPower(-1);

            if (shootTimer.seconds() > 0.35) {
                transfer.setPower(0);
                bt.outtakeElementPos(bt.getOuttakePosition());
                bt.turnToNextOuttake();
                ballsShot++;
                shootTimer.reset();

                if (ballsShot >= 3) {
                    finishingLastShot = true;
                }
            }

            if (rotoAdjust && rotoAdjusting.seconds() > 0.1) {
                roto.setPower(0.1);
                rotoAdjust = false;
            }

            if (finishingLastShot && shootTimer.seconds() > 0.25) {
                finishingLastShot = false;
                shootingAll = false;
                transfer.setPower(0);
                bt.turnToStartIntake();
                autoIntaking = true;
                IN.setPower(-0.9);
                bt.setAutoMode(true);
            }
        }
    }

    private void runManualShootMode(BallTrackerV2 bt) {
        motor.setVelocity(2200);

        if (gamepad2.bWasPressed() && !aPressed) {
            aPressed = true;
            pauseActive = true;
            pauseTimer.reset();

            bt.ballpos.remove(1);
            bt.ballpos.add(1, 0);
            bt.turnToNextOuttake();
            transfer.setPower(0);
        }

        if (pauseActive && pauseTimer.seconds() >= PAUSE_DURATION) {
            transfer.setPower(-1);
            counter--;
            delayTimer.reset();
            waiting = true;
            pauseActive = false;
        }

        if (waiting && delayTimer.seconds() >= DELAY_SECONDS) {
            transfer.setPower(0);
            waiting = false;
            aPressed = false;
        }
    }

    private void updateAutoIntake(BallTrackerV2 bt) {
        if (autoIntaking) {
            bt.updateAutoIntake();
            if (gamepad2.dpad_left) {
                autoIntaking = false;
                bt.setAutoMode(false);
            }
        }
    }

    private void updateIdleTransfer() {
        if (!shootingAll && !autoIntaking) {
            transfer.setPower(0);
        }
    }

    private void handleIntakeToggle() {
        if (gamepad2.yWasPressed()) {
            fuck4Balls = fuck4Balls * -1.0;
            IN.setPower(fuck4Balls);
        }
    }

    private void handleHoodPresets() {
        if (gamepad2.dpadUpWasPressed()) {
            servoTest.setPosition(0.0);
        }
        if (gamepad2.dpadDownWasPressed()) {
            servoTest.setPosition(0.11);
        }
    }

    private void handleFlywheelTrim() {
        if (gamepad2.right_bumper) {
            speed += 10;
        }
        if (gamepad2.left_bumper) {
            speed -= 10;
        }
    }
}
