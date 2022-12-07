package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.gonkaPipeline;


@Config
@Autonomous(group = "Competition")
public class redRightScore extends LinearOpMode {

    private final Pose2d startPose = new Pose2d(39, -63, Math.toRadians(90));
    private final Pose2d stackPose = new Pose2d(36, -12, Math.toRadians(90));

    private final double travelSpeed = 30.0, travelAccel = 15.0;
    private final double adjustmentSpeed = 1.5, adjustmentAccel = 1.5;

    SampleMecanumDrive drive;
    OpenCvWebcam camera = null;
    gonkaPipeline wiggleDetector = new gonkaPipeline();
    gonkaPipeline.wiggleDirection moveDir;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.stopAndResetMotors();
        drive.setGrip(false);
        drive.setSlideVelocity(0, drive.slideLeft, drive.slideRight, drive.slideTop);
        drive.stopAndResetMotors();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(36, -60, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
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

        camera.setPipeline(wiggleDetector);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();

        drive.setGrip(true);
        drive.setHeight(200);
        drive.setExtension(0);
        drive.restartMotors();
        sleep(500);
        drive.setSlideVelocity(400, drive.slideLeft, drive.slideRight);
        drive.setSlideVelocity(0, drive.slideTop);
        sleep(500);

        drive.setHeight(2500);
        drive.setSlideVelocity(1000, drive.slideLeft, drive.slideRight);

        drive.followTrajectorySequence(goToStack);
        drive.update();
        adjustAngle(drive);

        drive.setSlideVelocity(2000, drive.slideLeft, drive.slideRight);
        drive.setHeight(4200);
        sleep(500);
        drive.setSlideVelocity(1000, drive.slideTop);
        drive.setExtension(750);

        sleep(1000);
        drive.setGrip(false);

        sleep(500);
        drive.setHeight(100);
        drive.setExtension(50);
        drive.setSlideVelocity(1000, drive.slideLeft, drive.slideRight);
        drive.setSlideVelocity(750, drive.slideTop);
        sleep(5000);

        drive.stopAndResetMotors();

        while (true) {
            if (isStopRequested()) {
                break;
            }
        }



    }

    private void adjustAngle(SampleMecanumDrive _drive) {
        double degrees = 5.0;
        int tries = 2;
        gonkaPipeline.wiggleDirection prev = gonkaPipeline.wiggleDirection.STOP;
        while (moveDir != gonkaPipeline.wiggleDirection.STOP && degrees > 0 && tries > 0) {
            moveDir = wiggleDetector.getDirection();
            if (moveDir == gonkaPipeline.wiggleDirection.LEFT) {
                tries--;
                if (prev == moveDir) { degrees -= 2.5; }
                _drive.followTrajectorySequence(_drive.trajectorySequenceBuilder(_drive.getPoseEstimate()).turn(Math.toRadians(degrees)).build());
                prev = gonkaPipeline.wiggleDirection.LEFT;
                telemetry.addLine("on the left");
            } else if (moveDir == gonkaPipeline.wiggleDirection.RIGHT) {
                if (prev == moveDir) { degrees -= 2.5; }
                _drive.followTrajectorySequence(_drive.trajectorySequenceBuilder(_drive.getPoseEstimate()).turn(Math.toRadians(-degrees)).build());
                prev = gonkaPipeline.wiggleDirection.RIGHT;
                telemetry.addLine("on the right");
            }
            telemetry.update();
            if (isStopRequested()) {
                break;
            }
        }
    }

}