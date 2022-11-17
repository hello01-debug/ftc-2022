package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.advancedSlideControl;
import org.opencv.core.Mat;

@Config
@Autonomous(group = "Competition")
public class redRightScore extends LinearOpMode {
    private advancedSlideControl armControl;
    private final int[] startPos = {39, -63, 90};
    private final int[] stackPos = {36, -12, 45};

    private final double travelSpeed = 5.0, travelAccel = 1.5;
    private final double adjustmentSpeed = 1.5, adjustmentAccel = 1.5;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(startPos[0], startPos[1], Math.toRadians(startPos[2])));

        Trajectory goToFirst = drive.trajectoryBuilder(new Pose2d(startPos[0], startPos[1], Math.toRadians(startPos[2])))
                .lineTo(
                        new Vector2d(stackPos[0], stackPos[1]),
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .build();
        Trajectory left = drive.trajectoryBuilder(new Pose2d(53, 39, Math.toRadians(90)))
                .strafeLeft(5)
                .build();

        waitForStart();

        drive.followTrajectory(goToFirst);
        //drive.followTrajectory(left);


    }
}