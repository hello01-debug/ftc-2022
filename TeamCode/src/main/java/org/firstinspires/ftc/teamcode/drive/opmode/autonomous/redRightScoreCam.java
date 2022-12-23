package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.poleFinder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "Red Right Score w/ Camera")
public class redRightScoreCam extends LinearOpMode {

    private final Pose2d startPose = new Pose2d(39, -63, Math.toRadians(90));
    private final Pose2d stackPose = new Pose2d(36, -12, Math.toRadians(90));

    private final double travelSpeed = 45.0, travelAccel = 20.0;
    private final double adjustmentSpeed = 1.5, adjustmentAccel = 1.5;
    private final double angVel = Math.toRadians(180), adjustAngVel = Math.toRadians(20);

    private final int width = 1280, height = 720, slices = 64;

    SampleMecanumDrive drive;
    OpenCvWebcam camera = null;
    // poleFinder poleFinderPipeline = new poleFinder(width, height, slices);
    poleFinder poleFinderPipeline = new poleFinder();
    poleFinder.poleLocation moveDir;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        // Initialize arm
        drive.stopAndResetMotors();
        drive.setGrip(false);
        drive.setSlideVelocity(0, drive.slideLeft, drive.slideRight, drive.slideTop);

        drive.setPoseEstimate(startPose);

        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(3)
                .lineToLinearHeading(stackPose,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                //.turn(-Math.atan2(drive.getPoseEstimate().getY()-24, drive.getPoseEstimate().getX()-0) - Math.toRadians(drive.getPoseEstimate().getHeading()))
                .turn(Math.toRadians(45))
                .build();

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.setPipeline(poleFinderPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();

        drive.setGrip(true);
        drive.setHeight(200);
        drive.setExtension(0);
        drive.setSlideVelocity(400, drive.slideLeft, drive.slideRight);
        drive.restartMotors();
        sleep(250);

        drive.setHeight(2500);
        drive.setSlideVelocity(1000, drive.slideLeft, drive.slideRight);

        drive.followTrajectorySequence(goToStack);
        drive.updatePoseEstimate();
        adjustAngle(drive);

        drive.setSlideVelocity(2000, drive.slideLeft, drive.slideRight);
        drive.setHeight(4200);
        sleep(500);
        drive.setSlideVelocity(1000, drive.slideTop);
        drive.setExtension(700);

        sleep(1500);
        drive.setGrip(false);
        sleep(500);

        drive.setExtension(100);
        drive.setHeight(100);
        drive.updatePoseEstimate();
        drive.setSlideVelocity(1000, drive.slideLeft, drive.slideRight, drive.slideTop);
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(40, -10, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(40), DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20)
                )
                .build()
        );
        drive.updatePoseEstimate();
        drive.setHeight(750);
        drive.setExtension(1950);
        sleep(1500);
        drive.setGrip(true);
        sleep(500);

        drive.setHeight(2500);
        drive.setSlideVelocity(2000, drive.slideLeft, drive.slideRight);
        sleep(500);
        drive.setExtension(0);

        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(135)),
                        SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(40), DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20)
                )
                .build()
        );

        drive.updatePoseEstimate();
        adjustAngle(drive);

        drive.setSlideVelocity(2000, drive.slideLeft, drive.slideRight);
        drive.setHeight(4200);
        sleep(500);
        drive.setSlideVelocity(1000, drive.slideTop);
        drive.setExtension(700);

        sleep(1000);
        drive.setGrip(false);
        sleep(500);

        while (true) {
            if (isStopRequested()) {
                break;
            }
        }
    }

    private void adjustAngle(SampleMecanumDrive _drive) {
        double degrees = 2.0;
        int tries = 1;
        poleFinder.poleLocation prev = poleFinder.poleLocation.ALIGNED;

        while (moveDir != poleFinder.poleLocation.ALIGNED && tries > 0) {
            moveDir = poleFinderPipeline.getLocation();
            if (moveDir == poleFinder.poleLocation.LEFT) {
                if (prev == moveDir) {
                    tries -= 1;
                    degrees -= 1.0;
                }
                _drive.followTrajectorySequence(_drive.trajectorySequenceBuilder(_drive.getPoseEstimate())
                        .turn(Math.toRadians(degrees))
                        .build());
                prev = poleFinder.poleLocation.LEFT;
            } else if (moveDir == poleFinder.poleLocation.RIGHT) {
                if (prev == moveDir) {
                    tries -= 1;
                    degrees -= 1.0;
                }
                _drive.followTrajectorySequence(_drive.trajectorySequenceBuilder(_drive.getPoseEstimate())
                        .turn(Math.toRadians(-degrees))
                        .build());
                prev = poleFinder.poleLocation.RIGHT;
            }
            if (isStopRequested()) {
                break;
            }
        }
    }
}
