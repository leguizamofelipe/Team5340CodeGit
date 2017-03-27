package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.io.FileNotFoundException;

/**
 * Created by RobotAdmin on 3/24/2017.
 */
@Autonomous(name="Drive Straight", group="Pushbot")
@Disabled
public class TestStraightDrive extends CommonFunctions {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomyMotorAndSensorSetup();
        ///////////LOGGER SETUP/////////////////////////
        try {
            Logger = new Logger(true, "DriveStraight");
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        waitForStart();
        DriveForwardWithEncoder(100, (0.5));
    }
}