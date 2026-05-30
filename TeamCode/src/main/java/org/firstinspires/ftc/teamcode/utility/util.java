package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareInit;

/**
 * Base class for utility modules that need direct hardware access.
 * Subclasses inherit all motors, sensors, and telemetry from {@link HardwareInit}.
 */
public class util extends HardwareInit {

    public util(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
    }
}
