package org.firstinspires.ftc.teamcode.drive.opmode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "NewManualOp")
public class NewManualOp extends OpMode {
    private Robot robot;
    private Controller controller1, controller2;

    public void init() {
        robot = new Robot(hardwareMap, telemetry);
    }
}
