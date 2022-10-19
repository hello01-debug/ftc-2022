package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")
public class Autonomous_Graeme extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // this is our robot drive represents all the moters servos -coponets
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        // this is just an infinite look looking 
        while ( opModeIsActive() && !isStopRequested()){};


    }
}
