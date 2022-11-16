package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Mat;

@Config
@Autonomous(group = "Competition")
public class redRightScore extends LinearOpMode {
    public int startX = 39, startY = -63, startR = 90;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(startX, startY, Math.toRadians(startR)));

        Trajectory goToFirst = drive.trajectoryBuilder(new Pose2d(36, -12, Math.toRadians(startR)))
                .forward(1)
                .build();
        Trajectory left = drive.trajectoryBuilder(new Pose2d(53, 39, Math.toRadians(90)))
                .strafeLeft(5)
                .build();

        waitForStart();

        drive.followTrajectory(goToFirst);
        //drive.followTrajectory(left);
    }
}