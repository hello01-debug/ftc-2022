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

    private Servo leftGripServo, rightGripServo;
    private DcMotorEx slideLeft, slideRight, slideTop;

    @Override
    public void runOpMode() throws InterruptedException {

        leftGripServo = hardwareMap.servo.get("leftGripServo");
        rightGripServo = hardwareMap.servo.get("rightGripServo");

        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");
        slideTop = hardwareMap.get(DcMotorEx.class, "slideTop");

        stopAndResetMotors();
        setGrip(false);
        setSlideVelocity(0, slideLeft, slideRight, slideTop);
        stopAndResetMotors();

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

        setGrip(true);
        setHeight(200);
        setExtension(0);
        restartMotors();
        sleep(500);
        setSlideVelocity(400, slideLeft, slideRight);
        setSlideVelocity(0, slideTop);
        sleep(500);

        setHeight(4200);
        setSlideVelocity(1000, slideLeft, slideRight);

        drive.followTrajectorySequence(goToStack);

        setExtension(750);
        setSlideVelocity(1000, slideTop);

        sleep(1000);
        setGrip(false);

        sleep(500);
        setHeight(100);
        setExtension(50);
        setSlideVelocity(1000, slideLeft, slideRight);
        setSlideVelocity(750, slideTop);
        sleep(5000);

        stopAndResetMotors();

        while (true) {
            if (isStopRequested()) {
                break;
            }
        }

    }

    private void setMotorMode(DcMotorEx.RunMode mode, DcMotorEx... motors) {
        // Iterate over each DcMotor object and set their motor mode
        for (DcMotorEx motor : motors) {
            motor.setMode(mode);
        }
    }

    public void stopAndResetMotors() {
        setMotorMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER, slideLeft, slideRight, slideTop);
    }

    public void restartMotors() {
        setMotorMode(DcMotorEx.RunMode.RUN_TO_POSITION, slideLeft, slideRight, slideTop);
    }

    public void setHeight(int height) {
        slideLeft.setTargetPosition(height);
        slideRight.setTargetPosition(-height);
    }

    public void setExtension(int ext) {
        slideTop.setTargetPosition(-ext);
    }

    public void setSlideVelocity(int vel, DcMotorEx... motors) {
        for (DcMotorEx motor : motors) {
            motor.setVelocity(vel);
        }
    }

    public void setGrip(boolean grip) {
        double gripPower = grip ? 0 : 1;
        double leftPos = ((-1 * gripPower + 1) / 3) + 5.0/9;
        double rightPos = (gripPower / 3) + 2.0/3;

        leftGripServo.setPosition(leftPos);
        rightGripServo.setPosition(rightPos);
    }
}