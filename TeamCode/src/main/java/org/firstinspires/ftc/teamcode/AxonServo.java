package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.AutoFunctions;

public class AxonServo { // Axon servo utility
    private final double[] positions = {0.55, 1.65, 2.75};
    private int index = 0;
    private AutoFunctions auto;

    public void servoRotate(CRServo servo, AnalogInput ai, double angle, double speed) {
        if (angle < 360 && angle > -360) {
            DcMotorSimple.Direction direction;
            if (Math.signum(angle) == -1) {
                direction = DcMotorSimple.Direction.FORWARD;
            } else {
                direction = DcMotorSimple.Direction.REVERSE;
            }
            servo.setDirection(direction);
            servo.setPower(speed);
            double startVoltage = ai.getVoltage();
            double targetVoltage =
                    ((startVoltage + ((angle / 360) * 3.3)) % 3.3);
            double currentVoltage = startVoltage;
            while (!(targetVoltage - 0.05 < currentVoltage && currentVoltage < targetVoltage + 0.05)) {
                currentVoltage = ai.getVoltage();
            }
            servo.setPower(0);
        } else {
            int halfRotations = Math.toIntExact(Math.floorDiv(
                    Math.round(angle),
                    180));
            double extra = angle % (halfRotations * 180);
            for (int i = 0; i < halfRotations; i++) {
                servoRotate(servo, ai, 180, speed);
            }
            servoRotate(servo, ai, extra, speed);
        }
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void rotate(CRServo servo, AnalogInput ai, Telemetry tele, double voltage, double speed) {
        DcMotorSimple.Direction direction = (voltage - ai.getVoltage() < 0) ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
        servo.setDirection(direction);

        while (Math.abs(ai.getVoltage() - voltage) > 0.05) {
            servo.setPower(auto.adjustError(speed, Math.abs(ai.getVoltage() - voltage), 0.7));
            tele.addData("Voltage:",ai.getVoltage());
            tele.update();
        }
        servo.setPower(0);
    }

    public void switchNextPosition(CRServo servo, AnalogInput ai, double speed) {
        double target = positions[index];
        double currentVoltage = ai.getVoltage();

        DcMotorSimple.Direction direction = (target - currentVoltage < 0) ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
        servo.setDirection(direction);

        while (Math.abs(ai.getVoltage() - target) > 0.05) {
            servo.setPower(speed);
        }

        servo.setPower(0);
        index = (index + 1) % positions.length;
    }

}
