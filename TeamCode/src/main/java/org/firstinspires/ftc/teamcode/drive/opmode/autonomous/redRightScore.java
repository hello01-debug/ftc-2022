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

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;

@Config
@Autonomous(group = "Competition")
public class redRightScore extends LinearOpMode {

    private final Pose2d startPose = new Pose2d(39, -63, Math.toRadians(90));
    private final Pose2d stackPose = new Pose2d(36, -12, Math.toRadians(90));

    private final double travelSpeed = 15.0, travelAccel = 15.0;
    private final double adjustmentSpeed = 1.5, adjustmentAccel = 1.5;

    SampleMecanumDrive drive;

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
                .lineToLinearHeading(stackPose,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .turn(-Math.atan2(drive.getPoseEstimate().getY()-24, drive.getPoseEstimate().getX()-0) - Math.toRadians(drive.getPoseEstimate().getHeading()))
                .build();

        waitForStart();

        drive.setGrip(true);
        drive.setHeight(200);
        drive.setExtension(0);
        drive.restartMotors();
        sleep(500);
        drive.setSlideVelocity(400, drive.slideLeft, drive.slideRight);
        drive.setSlideVelocity(0, drive.slideTop);
        sleep(500);

        drive.setHeight(4200);
        drive.setSlideVelocity(1000, drive.slideLeft, drive.slideRight);

        drive.followTrajectorySequence(goToStack);

        drive.setExtension(750);
        drive.setSlideVelocity(1000, drive.slideTop);

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


}