package org.firstinspires.ftc.teamcode.drive.opmode.manual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.Controller;
import org.firstinspires.ftc.teamcode.drive.opmode.Robot;

/**
 * Mecanum teleop (with an optional headless mode)
 * * Left stick controls x/y translation.
 * * Triggers control rotation about the z axis
 * * When headless mode is enabled (press "cross"), translation direction
 * becomes relative to the field as opposed to the robot. You can
 * reset the forward heading by pressing "square".
 */
@TeleOp(name = "HeadlessOp")
public class HeadlessOp extends OpMode {
    // Create new robot and controller objects from the classes we defined
    private Robot robot;
    private Controller controller1, controller2;

    // Set up some useful variables
    private boolean headlessMode = false;   // Allows us to toggle headless mode on and off
    private int gyroCalibratedCount = 0;    // I don't know what this actually does
    private boolean gripperStowed = false;  // Allows use to stow and unstow the gripper
    private boolean grip = false;                // Controls how far open or closed the gripper is
    private double multiplier = 1.0;       // Allows us to slow down the motor speed

    @Override
    public void init() {
        // Basic setup
        robot = new Robot(hardwareMap, telemetry);
        robot.runUsingEncoders();
        robot.runSlideWithoutEncoders();
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
    }

    @Override
    public void init_loop() {
        // Check for controller updates
        controller1.update();
        controller2.update();

        // Toggle headless mode when cross is pressed on base driver's controller
        if (controller1.crossOnce()) {
            headlessMode = ! headlessMode;
        }

        // Stow/unstow gripper when cross is pressed on arm driver's controller
        if (controller2.crossOnce()) {
            gripperStowed = !gripperStowed;
        }

        // Add some telemetry information for convenience
        telemetry.addData("Gyro Ready?", robot.isGyroCalibrated() ? "yes" : "no");
        telemetry.addData("Headless Mode (cross)", headlessMode ? "yes" : "no");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Check controllers for updates
        // Update robot heading
        controller1.update();
        controller2.update();
        robot.loop();

        // Reset heading when base driver presses square
        if (controller1.squareOnce()) {
            robot.resetHeading();
        }

        // Toggle headless when base driver presses cross
        if (controller1.crossOnce()) {
            headlessMode = !headlessMode;
        }

        // Change multiplier when base driver presses circle
        if (controller1.circleOnce()) {
            multiplier = multiplier == 1.0 ? 0.25 : 1.0;
        }

        // Set grippers to open when the left bumper is pressed
        if (controller2.leftTriggerOnce()) {
            grip = false;
        }
        // Set grippers to close when the right bumper is pressed
        if (controller2.rightTriggerOnce()) {
            grip = true;
        }

        // Add telemetry for convenience
        telemetry.addData("Headless Mode (cross)", headlessMode ? "yes" : "no");
        telemetry.addData("Heading (reset: square)", robot.getHeadingDegrees());
        telemetry.update();

        if (controller2.Circle() || controller2.Cross() || controller2.Square() || controller2.Triangle()) {
            if (controller2.Triangle()) {
                robot.setMotors(0.1f, 0.1f, 0.1f, 0.1f, 1);
            }
            if (controller2.Cross()) {
                robot.setMotors(-0.1f, -0.1f, -0.1f, -0.1f, 1);
            }
            if (controller2.Square()) {
                robot.setMotors(-0.1f, 0.1f, 0.1f, -0.1f, 1);
            }
            if (controller2.Circle()) {
                robot.setMotors(0.1f, -0.1f, -0.1f, 0.1f, 1);
            }
        }
        else if (controller2.dpadDown() || controller2.dpadUp() || controller2.dpadLeft() || controller2.dpadRight()) {
            if (controller2.dpadUp()) {
                robot.setMotors(1.5f, 1.5f, 1.5f, 1.5f, 1);
            }
            if (controller2.dpadDown()) {
                robot.setMotors(-1.5f, -1.5f, -1.5f, -1.5f, 1);
            }
            if (controller2.dpadLeft()) {
                robot.setMotors(-1.5f, 1.5f, 1.5f, -1.5f, 1);
            }
            if (controller2.dpadRight()) {
                robot.setMotors(1.5f, -1.5f, -1.5f, 1.5f, 1);
            }
        }
        else if (controller2.rightBumper()) {
            robot.setMotors(0.15f, 0.15f, -0.15f, -0.15f, 1);
        }
        else if (controller2.leftBumper()) {
            robot.setMotors(-0.15f, -0.15f, 0.15f, 0.15f, 1);
        }
        else {
            // Get input from the base driver's left stick and take it to the third power to increase low-range resolution
            final double x = -Math.pow(controller1.left_stick_x, 3.0);
            final double y = Math.pow(controller1.left_stick_y, 3.0);
            final double rotation = Math.pow(controller1.right_trigger - controller1.left_trigger, 3.0);

            // get direction as the counterclockwise angle from the x-axis to point (x, y)
            // then compensate for reference angle by adding current heading
            final double direction = Math.atan2(x, y) + (headlessMode ? robot.getHeading() : 0.0);
            // determine speed using pythagorean theorem
            final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

            // another attempt at making sense of this headless stuff
            // basically splitting apart everything into basic x and y again, but with direction, then doing it like normal? idk man it's 3am and i'm confused
            final double y_proc = -1 * speed * Math.sin(direction + Math.PI / 2.0);
            final double x_proc = speed * Math.cos(direction + Math.PI / 2.0);

            // This is Mecanum stuff, I'll do my best to explain
        /*
            To move forward and backward, you apply the same rotation to all motors
            To move side to side, each pair of diagonal motors moves together, but the motors on each side move opposite each other
            The rotation is different from normal for some strange reason. I have no clue why. Sorry.
         */
            final double leftFront = y_proc + x_proc + rotation;
            final double leftRear = y_proc - x_proc - rotation;
            final double rightFront = y_proc - x_proc + rotation;
            final double rightRear = y_proc + x_proc - rotation;

            // Set all of the drive motors
            robot.setMotors(leftFront, rightFront, leftRear, rightRear, multiplier);
        }

        // Set up all of the arm values
        final double slideLeft = Math.pow(controller2.left_stick_y, 3.0);
        final double slideRight = Math.pow(controller2.left_stick_y, 3.0);
        final double slideTop = Math.pow(controller2.right_stick_y, 3.0);
        final boolean gripPower = grip;

        // Apply power to slide motors and gripper
        robot.setSlideMotors(slideLeft, slideRight, slideTop);
        robot.setGrip(gripPower, gripperStowed);

    }
}