package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LimeLight {
    public Limelight3A limelight;
    public double tx;
    public double ty;
    public double distance;
    public SlidingAverage txAvg;
    public SlidingAverage tyAvg;
    public SlidingAverage distanceAvg;
    public double txAvgVal;
    public double tyAvgVal;
    public double distanceAver;
    public double cameraHeight = 16.0;
    public double apriltagHeight = 30.0;
    public double cameraAngle = 0.0;
    public Telemetry telemetry;

    public LimeLight(Limelight3A ll, Telemetry tele) {
        limelight = ll;
        telemetry = tele;
        limelight.pipelineSwitch(0);
        limelight.start();
        txAvg = new SlidingAverage(5);
        tyAvg = new SlidingAverage(5);
        distanceAvg = new SlidingAverage(5);
        telemetry.addLine("Limelight");

    }

    public void loop() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            tx = result.getTx();
            ty = result.getTy();
            distance = calculateDistance(ty);
            txAvgVal = txAvg.update(tx);
            tyAvgVal = tyAvg.update(ty);
            distanceAver = distanceAvg.update(distance);
            telemetry.addData("tx: ", txAvgVal);
            telemetry.addData("ty:  ", tyAvgVal);
            telemetry.addData("Distance: ", distanceAver);
        } else {
            tx = ty = distance = Double.NaN;
            txAvgVal = tyAvgVal = distanceAver = Double.NaN;
            txAvg.reset();
            tyAvg.reset();
            distanceAvg.reset();
            telemetry.addLine("No Target Perceived");
        }
    }
    private double calculateDistance(double ty) {
        double angle = Math.toRadians(cameraAngle + ty);
        return (apriltagHeight - cameraHeight) / Math.tan(angle);
    }
}
