package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;

import java.io.FileNotFoundException;

@Autonomous(name = "TestLineTracking", group = "Tests")

public class TestLineTracking extends CommonFunctions {


    @Override
    public void runOpMode() throws InterruptedException {
        try {
            Logger = new Logger(true,"Track Line Inwards");
        }catch (FileNotFoundException e) {

            e.printStackTrace();
        }

        AutonomyMotorAndSensorSetup();

        // wait for the start button to be pressed.
        waitForStart();

        TrackLineInwards();

    }
}
