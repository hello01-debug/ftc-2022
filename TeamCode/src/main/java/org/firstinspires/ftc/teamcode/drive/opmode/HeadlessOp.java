package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Mecanum teleop (with an optional headless mode)
 * * Left stick controls x/y translation.
 * * Right stick controls rotation about the z axis
 * * When headless mode is enabled (press "cross"), translation direction
 * becomes relative to the field as opposed to the robot. You can
 * reset the forward heading by pressing "square".
 */
// TODO: Reset name
@TeleOp(name = "HeadlessOp - Do Not Use")
public class HeadlessOp extends OpMode {
    private Robot robot;
    private Controller controller;
    private boolean headlessMode = false;
    private int gyroCalibratedCount = 0;
    private boolean brakes = true;
    private boolean controlScheme = true;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.runWithoutEncoders();
        //robot.runWithBrakes();
        controller = new Controller(gamepad1);
    }

    @Override
    public void init_loop() {
        controller.update();
        if (controller.crossOnce()) {
            headlessMode = ! headlessMode;
        }
        if (controller.triangleOnce()) {
            brakes = !brakes;
        }
        if (controller.circleOnce()) {
            controlScheme = ! controlScheme;
        }
        telemetry.addData("Gyro Ready?", robot.isGyroCalibrated() ? "YES" : "no.");
        telemetry.addData("Headless Mode (cross)", headlessMode ? "YES" : "no.");
        telemetry.addData("Brakes: ", brakes ? "on" : "off");
        telemetry.addData("Control scheme ", controlScheme ? "1" : "2");
        telemetry.update();
    }

    @Override
    public void loop() {
        controller.update();
        robot.loop();
        if (brakes) {
            robot.runWithBrakes();
        }
        if (!brakes) {
            robot.runWithoutBrakes();
        }

        if (controller.squareOnce()) {
            robot.resetHeading();
        }
        if (controller.crossOnce()) {
            headlessMode = !headlessMode;
        }
        if (controller.triangleOnce()) {
            brakes = !brakes;
        }
        if (controller.circleOnce()) {
            controlScheme = ! controlScheme;
        }
        telemetry.addData("Headless Mode (cross)", headlessMode ? "YES" : "no.");
        telemetry.addData("Heading (reset: square)", robot.getHeadingDegrees());
        telemetry.addData("Brakes: ", brakes ? "on" : "off");
        telemetry.addData("Control scheme ", controlScheme ? "1" : "2");
        telemetry.update();

        final double x = -Math.pow(controlScheme ? controller.right_stick_x : controller.left_stick_x, 3.0);
        final double y = Math.pow(controller.left_stick_y, 3.0);

        final double rotation = Math.pow(controller.right_trigger-controller.left_trigger, 3.0);
        // get direction as the counterclockwise angle from the x-axis to point (x, y)
        // then compensate for reference angle by adding current heading
        final double direction = Math.atan2(x, y) + (headlessMode ? robot.getHeading() : 0.0);
        // determine speed using pythagorean theorem
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        // another attempt at making sense of this headless stuff
        // basically splitting apart everything into basic x and y again, but with direction, then doing it like normal? idk man it's 3am and i'm confused
        final double y_proc = -1 * speed * Math.sin(direction + Math.PI / 2.0);
        final double x_proc = speed * Math.cos(direction + Math.PI / 2.0);

        final double leftFront = y_proc + x_proc + rotation;
        final double leftRear = y_proc - x_proc - rotation;
        final double rightFront = y_proc - x_proc + rotation;
        final double rightRear = y_proc + x_proc - rotation;

        robot.setMotors(leftFront, rightFront, leftRear, rightRear);
    }
}