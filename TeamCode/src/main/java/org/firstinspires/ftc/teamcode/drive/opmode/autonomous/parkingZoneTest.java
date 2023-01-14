package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.parkingZoneFinder;


@Config
@Autonomous
public class parkingZoneTest extends LinearOpMode {
    SampleMecanumDrive drive;
    OpenCvWebcam camera = null;
    parkingZoneFinder parkingZonePipeline = new parkingZoneFinder();
    parkingZoneFinder.parkingZone zone = parkingZoneFinder.parkingZone.UNKNOWN;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.setPipeline(parkingZonePipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        while (true) {
            zone = parkingZonePipeline.getParkingZone();
            if (isStarted()) { break; }
        }

        if (zone == parkingZoneFinder.parkingZone.ZONE1) {
            telemetry.addLine("Zone 1");
        } else if (zone == parkingZoneFinder.parkingZone.ZONE2) {
            telemetry.addLine("Zone 2");
        } else if (zone == parkingZoneFinder.parkingZone.ZONE3) {
            telemetry.addLine("Zone 3");
        } else {
            telemetry.addLine("Unknown");
        }
    }
}
