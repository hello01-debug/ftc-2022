package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.FancyThresholdPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class gonkaCamera extends OpMode {

    OpenCvWebcam camera = null;

    @Override
    public void init() {

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.setPipeline(new gonkaPipeline());

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {
            }
        });
    }

    @Override
    public void loop() {

    }

    public class gonkaPipeline extends OpenCvPipeline {
        Mat YcBcr = new Mat();
        Mat output = new Mat();
        Mat[] verticalMat = new Mat[20];
        Rect[] verticalSlices = new Rect[20];
        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);
        Scalar[] verticalAvg = new Scalar[20];
        Scalar lower = new Scalar(0.0, 138.8, 51.0);
        Scalar upper = new Scalar(255.0, 178.5, 94.9);
        Mat mask = new Mat();
        int left, right;

        public Mat processFrame(Mat input) {
            left = 0;
            right = 0;

            Imgproc.cvtColor(input, YcBcr, Imgproc.COLOR_RGB2YCrCb);

            Core.inRange(YcBcr, lower, upper, mask);

            YcBcr.copyTo(output);

            for (int currentRect = 0; currentRect < 20; currentRect++) {
                verticalSlices[currentRect] = new Rect((currentRect * 64), 1, 64, 719);
                verticalMat[currentRect] = mask.submat(verticalSlices[currentRect]);
                verticalAvg[currentRect] = Core.mean(verticalMat[currentRect]);
                if (verticalAvg[currentRect].val[0] > 128) {
                    Imgproc.rectangle(output, verticalSlices[currentRect], new Scalar(0.0, 255.0, 0.0), 2);
                    if (currentRect < 10) {
                        left++;
                    } else {
                        right++;
                    }
                } else {
                    Imgproc.rectangle(output, verticalSlices[currentRect], rectColor, 2);
                }
                if (left > right) {
                    Imgproc.putText(output,"Move Left", new Point(360, 360), Imgproc.FONT_HERSHEY_SIMPLEX, 5, new Scalar(0.0, 0.0, 255.0), 5);
                } else if (right > left) {
                    Imgproc.putText(output,"Move Right", new Point(360, 360), Imgproc.FONT_HERSHEY_SIMPLEX, 5, new Scalar(0.0, 0.0, 255.0), 5);
                }
            }

            // Basically i'll put up a mask that only lets yellow through, then it will check to see which square has the most visible (non-masked) pixels to see where the pole would be

            //Imgproc.putText(output, "test text", new Point(25, 100), Imgproc.FONT_HERSHEY_COMPLEX, 2.0, new Scalar(255.0, 0.0, 0.0));

            return output;

        }
    }

}
