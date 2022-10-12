package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller {
    private Gamepad gamepad;

    private int dpad_up, dpad_down, dpad_left, dpad_right;
    private int square, triangle, cross, circle;
    private int left_bumper, right_bumper;

    public double left_stick_x, right_stick_x, left_stick_y, right_stick_y;
    public double left_trigger, right_trigger;

    public Controller(Gamepad g) {
        gamepad = g;
    }

    public void update() {
        if (gamepad.square) {
            ++square;
        } else {
            square = 0;
        }
        if (gamepad.triangle) {
            ++triangle;
        } else {
            triangle = 0;
        }
        if (gamepad.cross) {
            ++cross;
        } else {
            cross = 0;
        }
        if (gamepad.circle) {
            ++circle;
        } else {
            circle = 0;
        }
        if (gamepad.dpad_up) {
            ++dpad_up;
        } else {
            dpad_up = 0;
        }
        if (gamepad.dpad_down) {
            ++dpad_down;
        } else {
            dpad_down = 0;
        }
        if (gamepad.dpad_left) {
            ++dpad_left;
        } else {
            dpad_left = 0;
        }
        if (gamepad.dpad_right) {
            ++dpad_right;
        } else {
            dpad_right = 0;
        }
        if (gamepad.left_bumper) {
            ++left_bumper;
        } else {
            left_bumper = 0;
        }
        if (gamepad.right_bumper) {
            ++right_bumper;
        } else {
            right_bumper = 0;
        }

        left_stick_x = gamepad.left_stick_x;
        left_stick_y = gamepad.left_stick_y;
        right_stick_x = gamepad.right_stick_x;
        right_stick_y = gamepad.right_stick_y;
        left_trigger = gamepad.left_trigger;
        right_trigger = gamepad.right_trigger;
    }

    public boolean dpadUp() {
        return 0 < dpad_up;
    }

    public boolean dpadDown() {
        return 0 < dpad_down;
    }

    public boolean dpadLeft() {
        return 0 < dpad_left;
    }

    public boolean dpadRight() {
        return 0 < dpad_right;
    }

    public boolean Square() {
        return 0 < square;
    }

    public boolean Triangle() {
        return 0 < triangle;
    }

    public boolean Cross() {
        return 0 < cross;
    }

    public boolean Circle() {
        return 0 < circle;
    }

    public boolean leftBumper() {
        return 0 < left_bumper;
    }

    public boolean rightBumper() {
        return 0 < right_bumper;
    }

    public boolean dpadUpOnce() {
        return 1 == dpad_up;
    }

    public boolean dpadDownOnce() {
        return 1 == dpad_down;
    }

    public boolean dpadLeftOnce() {
        return 1 == dpad_left;
    }

    public boolean dpadRightOnce() {
        return 1 == dpad_right;
    }

    public boolean squareOnce() {
        return 1 == square;
    }

    public boolean triangleOnce() {
        return 1 == triangle;
    }

    public boolean crossOnce() {
        return 1 == cross;
    }

    public boolean circleOnce() {
        return 1 == circle;
    }

    public boolean leftBumperOnce() {
        return 1 == left_bumper;
    }

    public boolean rightBumperOnce() {
        return 1 == right_bumper;
    }

}