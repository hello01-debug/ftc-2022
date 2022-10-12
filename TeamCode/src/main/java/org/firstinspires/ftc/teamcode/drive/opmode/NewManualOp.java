package org.firstinspires.ftc.teamcode.drive.opmode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "NewManualOp")
public class NewManualOp extends OpMode {
    private Robot robot;
    private Controller controller1, controller2;

    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.runUsingEncoders();
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
    }

    public void loop() {
        controller1.update();
        controller2.update();

        double y = controller1.left_stick_y;
        double x = controller1.left_stick_x;
        double rot = controller1.right_trigger - controller2.right_trigger;

        final double leftFront = y + x + rot;
        final double leftRear = y - x + rot;
        final double rightFront = y - x - rot;
        final double rightRear = y + x - rot;

        robot.setMotors(leftFront, leftRear, rightFront, rightRear);

        // need to implement slide in robot class before continuing
    }
}
