package org.firstinspires.ftc.teamcode.drive.opmode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
/*
public class gonkaPipeline extends OpenCvPipeline {
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
 */

public class gonkaPipeline extends OpenCvPipeline {
    Mat YcBcr = new Mat();
    Mat output = new Mat();
    Mat[] verticalMat = new Mat[20];
    Rect[] verticalSlices = new Rect[20];
    Scalar rectColor = new Scalar(255.0, 0.0, 0.0);
    Scalar[] verticalAvg = new Scalar[20];

    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, YcBcr, Imgproc.COLOR_RGB2YCrCb);

        input.copyTo(output);

        for (int currentRect = 0; currentRect < 20; currentRect++) {
            verticalSlices[currentRect] = new Rect((currentRect * 64), 1, 64, 719);
            Imgproc.rectangle(output, verticalSlices[currentRect], rectColor, 2);
            verticalMat[currentRect] = YcBcr.submat(verticalSlices[currentRect]);
            verticalAvg[currentRect] = Core.mean(verticalMat[currentRect]);
            Imgproc.putText(output, Double.toString(verticalAvg[currentRect].val[0]), new Point((currentRect * 64 + 1), 200), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, rectColor);
        }

        // Basically i'll put up a mask that only lets yellow through, then it will check to see which square has the most visible (non-masked) pixels to see where the pole would be

        //Imgproc.putText(output, "test text", new Point(25, 100), Imgproc.FONT_HERSHEY_COMPLEX, 2.0, new Scalar(255.0, 0.0, 0.0));

        return output;

    }
}