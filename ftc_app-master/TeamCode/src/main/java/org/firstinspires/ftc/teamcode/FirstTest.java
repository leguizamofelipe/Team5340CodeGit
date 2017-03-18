package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by robotadmin on 3/16/2017.
 */

@Autonomous(name="InterceptLine and Gyro", group="Pushbot")
public class FirstTest extends CommonFunctions{
    final String AllianceColor = "RED";

    @Override
    public void runOpMode() throws InterruptedException {
        AutonomyMotorAndSensorSetup();

        waitForStart();

        while (gyro.isCalibrating()) {

        }

        DriveBackwardWithEncoder(10, 0.3);

        turnRight(70);

        DriveForwardWithEncoder(45, 0.4);

        AlignWithLine(AllianceColor, 0.17);

    }
}
