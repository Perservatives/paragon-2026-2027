package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// General use hardware items, for ease of access.

public class HardwareInit {

    // hardware initialization
    public final DcMotorEx fl, fr, bl, br;
    public final DcMotor intakeMotor;
    public final DcMotor roto;
    public final DcMotor transfer;
    public final DcMotorEx motor;
    public final Servo servoTest;
    //    public final CRServo selector;
    public final Servo selector;
    public final AnalogInput selectorInfo;
    public final IMU imu;
    public final GoBildaPinpointDriver odo;
    public TouchSensor test_magnetic;
    public final Telemetry tele;
    //public final Limelight3A limelight;
    public Limelight3A limelight;
    public final NormalizedColorSensor cs;
    public final VoltageSensor voltageSensor;

    public HardwareInit(HardwareMap hardwareMap, Telemetry telemetry) {
        tele = telemetry;

        // motor mapping
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        selector = hardwareMap.get(Servo.class, "selector");
        selector.setDirection(Servo.Direction.FORWARD);
        selector.setPosition(0);

        servoTest = hardwareMap.get(Servo.class, "servoTest");

        roto = hardwareMap.get(DcMotor.class, "turretMotor");
        roto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake"); // map name

        transfer = hardwareMap.get(DcMotor.class, "transfer");

        motor = hardwareMap.get(DcMotorEx.class, "motorTest");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);

        imu.initialize(parameters);

        imu.resetYaw();

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-6.65,0.79, DistanceUnit.INCH);
        odo.resetPosAndIMU();
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,GoBildaPinpointDriver.EncoderDirection.REVERSED);

        // analog
        selectorInfo = hardwareMap.get(AnalogInput.class, "selectorInfo");

        // limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // touch sensor
        test_magnetic = hardwareMap.get(TouchSensor.class, "testDis");

        // color sensor
        cs = hardwareMap.get(NormalizedColorSensor.class, "test_color");

        // voltage sensor
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    }
}