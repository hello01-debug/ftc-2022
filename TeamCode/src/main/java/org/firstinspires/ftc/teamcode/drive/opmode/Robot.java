package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Hardware definitions and access for a robot with a four-motor
 * drive train and a gyro sensor.
 */
public class Robot {
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private final DcMotor leftFront, leftRear, rightFront, rightRear;
    private final DcMotor slideLeft, slideRight, slideTop;
    private final BNO055IMU imu;
    private final Servo gripServo;

    private double headingOffset = 0.0;
    private Orientation angles;
    private Acceleration gravity;


    public Robot(final HardwareMap _hardwareMap, final Telemetry _telemetry) {
        hardwareMap = _hardwareMap;
        telemetry = _telemetry;

        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        slideLeft = hardwareMap.dcMotor.get("slideLeft");
        slideRight = hardwareMap.dcMotor.get("slideRight");
        slideTop = hardwareMap.dcMotor.get("slideTop");

        // One of the slide motors MUST be reversed or teh two motors will fight each other
        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gripServo = hardwareMap.servo.get("gripServo");
    }

    private void setMotorMode(DcMotor.RunMode mode, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setMode(mode);
        }
    }

    // TODO: add slide motors to this function once we have the encoders physically connected
    public void runUsingEncoders() {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER, leftFront, leftRear, rightFront, rightRear);
    }

    public void runWithoutEncoders() {
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, leftFront, leftRear, rightFront, rightRear);
    }

    private void setBrake(DcMotor.ZeroPowerBehavior mode, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(mode);
        }
    }

    public void runWithBrakes() {
        setBrake(DcMotor.ZeroPowerBehavior.BRAKE, leftFront, leftRear, rightFront, rightRear);
    }

    public void runWithoutBrakes() {
        setBrake(DcMotor.ZeroPowerBehavior.FLOAT, leftFront, leftRear, rightFront, rightRear);
    }

    /**
     * @return true if the gyro is fully calibrated, false otherwise
     */
    public boolean isGyroCalibrated() {
        return imu.isGyroCalibrated();
    }

    /**
     * Fetch all once-per-time-slice values.
     * <p>
     * Call this either in your OpMode::loop function or in your while(opModeIsActive())
     * loops in your autonomous. It refresh gyro and other values that are computationally
     * expensive.
     */
    public void loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        gravity = imu.getGravity();
    }

    /**
     * @return the raw heading along the desired axis
     */
    private double getRawHeading() {
        return angles.firstAngle;
    }

    /**
     * @return the robot's current heading in radians
     */
    public double getHeading() {
        return (getRawHeading() - headingOffset) % (2.0 * Math.PI);
    }

    /**
     * @return the robot's current heading in degrees
     */
    public double getHeadingDegrees() { return Math.toDegrees(getHeading()); }

    /**
     * Set the current heading to zero.
     */
    public void resetHeading() {
        headingOffset = getRawHeading();
    }

    /**
     * Find the maximum absolute value of a set of numbers.
     *
     * @param xs Some number of double arguments
     * @return double maximum absolute value of all arguments
     */
    private static double maxAbs(double... xs) {
        double ret = Double.MIN_VALUE;
        for (double x : xs) {
            if (Math.abs(x) > ret) {
                ret = Math.abs(x);
            }
        }
        return ret;
    }

    /**
     * Set motor powers
     * <p>
     * All powers will be scaled by the greater of 1.0 or the largest absolute
     * value of any motor power.
     *
     * @param _leftFront Left front motor
     * @param _leftRear Left rear motor
     * @param _rightFront Right front motor
     * @param _rightRear Right rear motor
     */
    public void setMotors(double _leftFront, double _leftRear, double _rightFront, double _rightRear) {
        final double scale = maxAbs(1.0, _leftFront, _leftRear, _rightFront, _rightRear);
        leftFront.setPower(_leftFront / scale);
        leftRear.setPower(_leftRear / scale);
        rightFront.setPower(_rightFront / scale);
        rightRear.setPower(_rightRear / scale);
    }

    public void setSlideMotors(double _slideLeft, double _slideRight, double _slideTop, double _gripServo) {
        slideLeft.setPower(_slideLeft);
        slideRight.setPower(_slideRight);
        slideTop.setPower(_slideTop);
        gripServo.setPosition(_gripServo);
    }
}