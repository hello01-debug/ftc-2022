package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class poleFinder extends OpenCvPipeline {
    Mat YCbCr = new Mat();
    Mat leftCrop, rightCrop;
    double leftavgfin, rightavgfin;
    Mat output = new Mat();
    Scalar rectColor = new Scalar(255.0, 0.0, 0.0);

    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
        //telemetry.addLine("pipeline running");

        Rect leftRect = new Rect(1, 1, 639, 719);
        Rect rightRect = new Rect(640, 1, 639, 719);

        input.copyTo(output);
        Imgproc.rectangle(output, leftRect, rectColor, 2);
        Imgproc.rectangle(output, rightRect, rectColor, 2);

        leftCrop = YCbCr.submat(leftRect);
        rightCrop = YCbCr.submat(rightRect);

        Core.extractChannel(leftCrop, leftCrop, 1);
        Core.extractChannel(rightCrop, rightCrop, 1);

        Scalar leftavg = Core.mean(leftCrop);
        Scalar rightavg = Core.mean(rightCrop);

        leftavgfin = leftavg.val[0];
        rightavgfin = rightavg.val[0];

        if (leftavgfin > rightavgfin) {
            //telemetry.addLine("left");
        } else {
            //telemetry.addLine("right");
        }

        return(output);
    }
}