package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Hardware definitions and access for a robot with a four-motor
 * drive train and a gyro sensor.
 */
public class advancedSlideControl {
    private final HardwareMap hardwareMap;

    // Create motor, servo, and gyro objects
    private final DcMotorEx slideLeft, slideRight, slideTop;

    // Create robot class
    public advancedSlideControl(final HardwareMap _hardwareMap) {
        // Pass variables through to Robot class
        hardwareMap = _hardwareMap;

        // Do the same thing we did earlier with the drive motors, just for the slide
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideleft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");
        slideTop = hardwareMap.get(DcMotorEx.class, "slideTop");
    }

    // This function accepts a motor mode followed by a list of DcMotor objects
    private void setMotorMode(DcMotorEx.RunMode mode, DcMotorEx... motors) {
        // Iterate over each DcMotor object and set their motor mode
        for (DcMotorEx motor : motors) {
            motor.setMode(mode);
        }
    }

    // Invoke setMotorMode() to turn on encoders
    public void stopAndResetMotors() {
        setMotorMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER, slideLeft, slideRight, slideTop);
    }

    // Invoke setMotorMode() to turn off encoders
    public void restartMotors() {
        setMotorMode(DcMotorEx.RunMode.RUN_TO_POSITION, slideLeft, slideRight, slideTop);
    }

    // Does the same thing as setMotorMode(), just with a zeroPowerMode
    private void setBrake(DcMotorEx.ZeroPowerBehavior mode, DcMotorEx... motors) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(mode);
        }
    }

    public void setHeight(int height) {
        slideLeft.setTargetPosition(height);
        slideLeft.setTargetPosition(-height);
    }

    public void setExtension(int ext) {
        slideTop.setTargetPosition(-ext);
    }

    public void setSlideVelocity(int vel, DcMotorEx... motors) {
        for (DcMotorEx motor : motors) {
            motor.setVelocity(vel);
        }
    }
}