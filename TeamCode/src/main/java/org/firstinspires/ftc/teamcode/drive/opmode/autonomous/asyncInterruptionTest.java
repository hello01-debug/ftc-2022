package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class asyncInterruptionTest extends LinearOpMode {

    SampleMecanumDrive drive;

    private final double maxAngVel = 3.0;
    private final double maxAngAccel = 3.0;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence turnLeft = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(90), maxAngVel, maxAngAccel)
                .build();

        TrajectorySequence turnRight = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(-90), maxAngVel, maxAngAccel)
                .build();

        TrajectorySequence doNothing = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(0), maxAngVel, maxAngAccel)
                .build();

        waitForStart();

        drive.followTrajectorySequenceAsync(turnLeft);

        sleep(5000);

        drive.followTrajectorySequence(turnRight);
    }
}
