package org.firstinspires.ftc.teamcode.drive.opmode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class parkingZoneFinder extends OpenCvPipeline {

    public enum parkingZone {
        ZONE1,
        ZONE2,
        ZONE3,
        UNKNOWN
    }

    // Scalars to define color ranges of interest
    public final Scalar oneLower = new Scalar(0.0, 128.0, 0.0);
    public final Scalar oneUpper = new Scalar(255.0, 255.0, 90.0);

    public final Scalar twoLower = new Scalar(0.0, 180.0, 0.0);
    public final Scalar twoUpper = new Scalar(255.0, 255.0, 150.0);

    public final Scalar threeLower = new Scalar(0.0, 43.0, 160.0);
    public final Scalar threeUpper = new Scalar(255.0, 106.0, 185.0);

    private Mat YcBcR = new Mat();
    private Mat oneMat = new Mat(), twoMat = new Mat(), threeMat = new Mat();
    private double oneAvg, twoAvg, threeAvg;

    private Mat output = new Mat();

    private int zoneNumber = -1;

    public Mat processFrame(Mat input) {
        //input.copyTo(output);
        Imgproc.cvtColor(input, YcBcR, Imgproc.COLOR_RGB2YCrCb);

        Core.inRange(YcBcR, oneLower, oneUpper, oneMat);
        Core.inRange(YcBcR, twoLower, twoUpper, twoMat);
        Core.inRange(YcBcR, threeLower, threeUpper, threeMat);

        oneAvg = Core.mean(oneMat).val[0];
        twoAvg = Core.mean(twoMat).val[0];
        threeAvg = Core.mean(threeMat).val[0];

        if (oneAvg > twoAvg && oneAvg > threeAvg) {
            zoneNumber = 1;
        } else if (twoAvg > oneAvg && twoAvg > threeAvg) {
            zoneNumber = 2;
        } else if (threeAvg > oneAvg && threeAvg > twoAvg) {
            zoneNumber = 3;
        }

        input.copyTo(output);

        Imgproc.putText(output, "Zone #" + String.valueOf(zoneNumber), new Point(25, 100), Imgproc.FONT_HERSHEY_SIMPLEX, 3.0, new Scalar(0.0, 255.0, 0.0));

        return output;
    }

    public parkingZone getParkingZone() {
        if (oneAvg > twoAvg && oneAvg > threeAvg) {
            return parkingZone.ZONE1;
        } else if (twoAvg > oneAvg && twoAvg > threeAvg) {
            return parkingZone.ZONE2;
        } else if (threeAvg > oneAvg && threeAvg > twoAvg) {
            return parkingZone.ZONE3;
        } else {
            return parkingZone.UNKNOWN;
        }
    }
}

/*
Color ranges for each parking zone

1: (0, 128, 0) (255, 255, 90)
2: (0, 180, 0) (255, 255, 150)
3: (0, 43, 160) (255, 106, 185)
 */