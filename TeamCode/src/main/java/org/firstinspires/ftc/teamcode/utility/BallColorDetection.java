package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

/**
 * Reads the color sensor and classifies detected balls.
 *
 * <p>Return values: {@code 0} = none, {@code 1} = purple, {@code 2} = green.</p>
 */
public class BallColorDetection {

    private static final double GREEN_MIN = 0.5;
    private static final double PURPLE_MIN = 0.2;
    private static final double ALPHA_MIN = 0.075;

    private final NormalizedColorSensor colorSensor;

    public BallColorDetection(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "test_color");
    }

    /**
     * Averages {@code cycles} color readings and returns the detected ball color.
     *
     * @param cycles number of sensor samples to average
     * @return {@code 0} if no ball, {@code 1} for purple, {@code 2} for green
     */
    public int getColor(int cycles) {
        double avgR = 0;
        double avgG = 0;
        double avgB = 0;

        for (int i = 0; i < cycles; i++) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            avgR += colors.red;
            avgG += colors.green;
            avgB += colors.blue;
        }

        avgR /= cycles;
        avgG /= cycles;
        avgB /= cycles;

        double sum = avgR + avgG + avgB;
        if (sum != 0) {
            avgR /= sum;
            avgG /= sum;
            avgB /= sum;
        }

        if (avgG > avgR && avgG > avgB && avgG > GREEN_MIN) {
            return 2;
        }
        if (avgB > avgG && avgR > PURPLE_MIN && avgB > PURPLE_MIN) {
            return 1;
        }
        return 0;
    }

    /**
     * Returns whether a ball is present based on averaged color readings.
     *
     * @param cycles number of sensor samples to average
     */
    public boolean ballExists(int cycles) {
        double avgR = 0;
        double avgG = 0;
        double avgB = 0;
        double avgAlpha = 0;

        for (int i = 0; i < cycles; i++) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            avgR += colors.red;
            avgG += colors.green;
            avgB += colors.blue;
            avgAlpha += colors.alpha;
        }

        avgR /= cycles;
        avgG /= cycles;
        avgB /= cycles;
        avgAlpha /= cycles;

        double sum = avgR + avgG + avgB;
        if (sum != 0) {
            avgR /= sum;
            avgG /= sum;
            avgB /= sum;
        }

        if ((avgG > avgR && avgG > avgB && avgG > GREEN_MIN) || avgAlpha > ALPHA_MIN) {
            return true;
        }
        return avgB > avgG && avgR > PURPLE_MIN && avgB > PURPLE_MIN;
    }
}
