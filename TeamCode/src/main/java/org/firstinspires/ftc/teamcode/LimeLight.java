package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LimeLight {
    public Limelight3A limelight;
    public volatile double tx;
    public volatile double ty;
    public double distance;
    public SlidingAverage txAvg;
    public SlidingAverage tyAvg;
    public SlidingAverage distanceAvg;
    public volatile double txAvgVal;
    public volatile double tyAvgVal;
    public volatile double distanceAver;
    public double cameraHeight = 16.0;
    public double apriltagHeight = 30.0;
    public double cameraAngle = 0.0;
    public Telemetry telemetry;

    public LimeLight(Limelight3A ll, Telemetry tele) {
        limelight = ll;
        telemetry = tele;
        limelight.pipelineSwitch(0);
        limelight.start();
        txAvg = new SlidingAverage(1);
        tyAvg = new SlidingAverage(2);
        distanceAvg = new SlidingAverage(3);
        telemetry.addLine("Limelight");

    }

    /** Polls Limelight and updates rolling averages (safe for background threads). */
    public void updateVision() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            tx = result.getTx();
            ty = result.getTy();
            distance = calculateDistance(ty);
            txAvgVal = txAvg.update(tx);
            tyAvgVal = tyAvg.update(ty);
            distanceAver = distanceAvg.update(distance);
        } else {
            tx = ty = distance = Double.NaN;
            txAvgVal = tyAvgVal = distanceAver = Double.NaN;
            // Do not reset averages — brief dropouts while moving should not wipe smoothing.
        }
    }

    public void loop() {
        updateVision();
        if (!Double.isNaN(txAvgVal)) {
            telemetry.addData("tx: ", txAvgVal);
            telemetry.addData("ty:  ", tyAvgVal);
            telemetry.addData("Distance: ", distanceAver);
        } else {
            telemetry.addLine("No Target Perceived");
        }
    }
    private double calculateDistance(double ty) {
        double angle = Math.toRadians(cameraAngle + ty);
        return (apriltagHeight - cameraHeight) / Math.tan(angle);
    }
}
