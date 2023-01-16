package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.poleFinder;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.parkingZoneFinder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "Red Right Score")
public class redRightScore extends LinearOpMode {

    private final Pose2d startPose = new Pose2d(36, -63, Math.toRadians(90));
    private final Pose2d scorePose = new Pose2d(40, -11, Math.toRadians(141));
    private final Pose2d stackPose = new Pose2d(40, -10, Math.toRadians(5));

    private final double travelSpeed = 45.0, travelAccel = 30.0;
    private final double adjustmentSpeed = 1.5, adjustmentAccel = 1.5;
    private final double angVel = Math.toRadians(120), adjustAngVel = Math.toRadians(20);

    private Pose2d[] parkingSpots = {new Pose2d(12, -15, Math.toRadians(90)), new Pose2d(36, -15, Math.toRadians(90)), new Pose2d(60, -15, Math.toRadians(90))};

    private final int width = 1280, height = 720, slices = 64;

    SampleMecanumDrive drive;
    OpenCvWebcam adjustCamera = null, signalCamera = null;
    parkingZoneFinder parkingZonePipeline = new parkingZoneFinder();
    parkingZoneFinder.parkingZone zone;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        // Initialize arm
        drive.initArm();

        // Tell the robot where it is based on a pose created earlier
        drive.setPoseEstimate(startPose);

        // Create the first trajectory to be run when the round starts

        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(scorePose,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .build();



        // Set up the webcam
        WebcamName adjustCameraName = hardwareMap.get(WebcamName.class, "adjustCamera");
        adjustCamera = OpenCvCameraFactory.getInstance().createWebcam(adjustCameraName);

        // Set the camera's pipeline
        adjustCamera.setPipeline(parkingZonePipeline);

        // Open the camera
        adjustCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                adjustCamera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        while (!isStarted()) {
            zone = parkingZonePipeline.getParkingZone();
            telemetry.addData("Parking Zone", zone);
            telemetry.update();
        }

        adjustCamera.stopStreaming();
        adjustCamera.closeCameraDevice();

        //waitForStart();

        drive.setSlideVelocity(4000, drive.slideRight, drive.slideLeft, drive.slideTop);

        // Close the grip and move the slide up a small amount
        drive.setGrip(true);
        sleep(250);
        drive.setHeight(200);
        drive.setExtension(50);

        // The sleep is necessary to wait for certain arm actions to finish
        sleep(250);

        // Increase the height of the slide and increase its velocity
        drive.setHeight(4200);
        drive.setExtension(825);

        drive.followTrajectorySequence(goToStack);

        // Without waiting, run the trajectory we prepared earlier
        // This will take us to our cycle location

        //  drive.followTrajectorySequence(goToStack);
        // Update roadrunner's idea of where the robot is after we ran the trajectory
        drive.updatePoseEstimate();
        // Adjust our angle so that we are lined up with the pole
        // Increase the slide height to high junction height and increase its velocity //TODO:
        // Wait until the slides are high enough that we won't hit the pole when extending
        sleep(250);
        // Extend the horizontal slide above the pole


        // Wait for arm to be in position

        // Open grip to drop cone
        drive.setGrip(false);

        // Wait for grip to fully open and cone to drop
        sleep(500);

        for (int i = 5; i > 2; i--) {
            toStack(drive, i);
            scoreCone(drive, i);
        }

        if (zone == parkingZoneFinder.parkingZone.ZONE1) { parkBot(drive, 0, parkingSpots); }
        else if (zone == parkingZoneFinder.parkingZone.ZONE2) { parkBot(drive, 1, parkingSpots); }
        else if (zone == parkingZoneFinder.parkingZone.ZONE3) { parkBot(drive, 2, parkingSpots); }
        else { parkBot(drive, 1, parkingSpots); }
    }
/*
    private void adjustAngle(SampleMecanumDrive _drive) {
        double degrees = 3.0;
        int tries = 1;
        poleFinder.poleLocation prev = poleFinder.poleLocation.ALIGNED;

        while (moveDir != poleFinder.poleLocation.ALIGNED && tries > 0) {
            moveDir = poleFinderPipeline.getLocation();
            if (moveDir == poleFinder.poleLocation.LEFT) {
                if (prev == moveDir) {
                    tries -= 3;
                    degrees -= 1.0;
                }
                _drive.followTrajectorySequence(_drive.trajectorySequenceBuilder(_drive.getPoseEstimate())
                        .turn(Math.toRadians(degrees), Math.toRadians(30.0), Math.toRadians(10.0))
                        .build());
                prev = poleFinder.poleLocation.LEFT;
            } else if (moveDir == poleFinder.poleLocation.RIGHT) {
                if (prev == moveDir) {
                    tries -= 1;
                    degrees -= 1.0;
                }
                _drive.followTrajectorySequence(_drive.trajectorySequenceBuilder(_drive.getPoseEstimate())
                        .turn(Math.toRadians(-degrees), Math.toRadians(30.0), Math.toRadians(10.0))
                        .build());
                prev = poleFinder.poleLocation.RIGHT;
            }
            if (isStopRequested()) {
                break;
            }
        }
    }
*/
    private void toStack(SampleMecanumDrive _drive, int stackHeight ) {
        // stackHeight is given as height of stack in cones
        //step one
        _drive.setExtension(700);

        int add = 5;

        if (stackHeight == 5) {
            add = 0;
        }

        _drive.updatePoseEstimate();
        TrajectorySequence turnToStack = _drive.trajectorySequenceBuilder(_drive.getPoseEstimate())
                .addTemporalMarker(0.25, () -> {
                    _drive.setHeight(225 + (stackHeight * 145));
                })
                .addTemporalMarker(0.50, () -> {
                    _drive.setExtension(1700);
                })
                .turn(Math.toRadians(-144 - add), Math.toRadians(120), Math.toRadians(90))
                .build();

        _drive.followTrajectorySequence(turnToStack);
        _drive.setExtension(2250);
        sleep(750);
        _drive.setGrip(true);
        sleep(450);
        //end of step two
        //start of step three
        _drive.setHeight(4100);
        sleep(500);
        //Start of step four
        _drive.setExtension(750);
    }

    private void scoreCone(SampleMecanumDrive _drive, int stackHeight) {
        int add = 0;

        if (stackHeight == 5) {
            add = 3;
        }
        _drive.updatePoseEstimate();
        TrajectorySequence reposition = _drive.trajectorySequenceBuilder(stackPose)
                .turn(Math.toRadians(141 - add), Math.toRadians(120), Math.toRadians(90))
                .build();


        _drive.setHeight(4100);

        _drive.followTrajectorySequence(reposition);

        _drive.setHeight(4100);

        _drive.setExtension(750); // Wait for wiggles to stop just in case

        // Increase the slide height to high junction height and increase its velocity //TODO:
        // Wait until the slides are high enough that we won't hit the pole when extending
        sleep(250);
        // Extend the horizontal slide above the pole

        // Wait for arm to be in position

        // Open grip to drop cone
        _drive.setGrip(false);
        sleep(250);
    }

    private void parkBot(SampleMecanumDrive _drive, int _zone, Pose2d[] locations) {
        _drive.updatePoseEstimate();
        Trajectory moveToPark = _drive.trajectoryBuilder(_drive.getPoseEstimate())
                .lineToLinearHeading(locations[_zone])
                .build();

        _drive.setGrip(false);
        _drive.setExtension(50);
        _drive.setHeight(4400);

        _drive.followTrajectory(moveToPark);

        _drive.setHeight(100);
    }
}
