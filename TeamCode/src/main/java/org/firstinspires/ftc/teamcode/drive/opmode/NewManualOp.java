package org.firstinspires.ftc.teamcode.drive.opmode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "NewManualOp")
public class NewManualOp extends OpMode {
    // Create new robot and controller objects from the classes we defined
    private Robot robot;
    private Controller controller1, controller2;
    private boolean cubicAccel = false;

    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.runUsingEncoders();
        robot.runWithBrakes();
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
    }

    public void init_loop() {
        // Check for controller updates
        controller1.update();
        // If cross is pressed, toggle cubic acceleration
        // Cubic acceleration curve lets us more smoothly control the robot than a linear curve
        if (controller1.crossOnce()) {
            cubicAccel = !cubicAccel;
        }
        telemetry.addData("Cubic acceleration: ", cubicAccel ? "yes" : "no");
    }

    public void loop() {
        // We aren't using the heading features of the robot in this opmode, so we only need to update the controllers
        controller1.update();
        controller2.update();

        if (controller1.crossOnce()) {
            cubicAccel = !cubicAccel;
        }
        telemetry.addData("Cubic acceleration: ", cubicAccel ? "yes" : "no");

        // If powerCurve is true, we will raise our inputs to the 3rd power, otherwise it will stay linear
        double powerCurve = cubicAccel ? 3.0 : 1.0;

        // TODO: make a function/method to do the power function for us a bit easier, takes to long as-is
        double y = -Math.pow(controller1.left_stick_y, powerCurve);
        double x = Math.pow(controller1.left_stick_x, powerCurve);
        double rot = Math.pow(controller1.right_trigger - controller1.left_trigger, powerCurve);

        // Put together a telemetry packet that shows our inputs, for diagnostic purposes
        telemetry.addData("X-input: ", x);
        telemetry.addData("Y-input: ", y);
        telemetry.addData("Rot. input ", rot);
        telemetry.update();

        final double leftFront = y + x + rot;
        final double leftRear = y - x + rot;
        final double rightFront = y - x - rot;
        final double rightRear = y + x - rot;

        robot.setMotors(leftFront, leftRear, rightFront, rightRear);

        double vert = Math.pow(controller2.left_stick_y, powerCurve);
        double grip = ((controller2.right_trigger - controller2.left_trigger) + 1) / 2;

        final double slideLeft = vert;
        final double slideRight = vert;
        final double slideTop = Math.pow(controller2.right_stick_x, powerCurve);
        final double gripServo = Math.pow(grip, powerCurve);

        robot.setSlideMotors(slideLeft, slideRight, slideTop, gripServo);

    }
}
