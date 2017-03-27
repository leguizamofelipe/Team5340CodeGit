package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.io.FileNotFoundException;

/**
 * Created by robotadmin on 3/16/2017.
 */

@Autonomous(name="TestInterceptLine", group="Tests")

public class TestInterceptLine extends CommonFunctions{
    final String AllianceColor = "BLUE";

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            Logger = new Logger(true,"TWORANGESENSORTEST");
        }catch (FileNotFoundException e) {

            e.printStackTrace();
        }

        AutonomyMotorAndSensorSetup();

        waitForStart();

        while (gyro.isCalibrating()) {

        }

        AlignWithLineUsingODS(AllianceColor, 0.17);

    }
}
