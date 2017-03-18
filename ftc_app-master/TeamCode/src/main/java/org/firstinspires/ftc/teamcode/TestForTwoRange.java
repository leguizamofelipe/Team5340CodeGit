package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by RobotAdmin on 3/15/2017.
 */
@Autonomous(name = "TestForTwoRange", group = "Concept")
public class TestForTwoRange extends CommonFunctions {

    @Override
    public void runOpMode() throws InterruptedException {
        AutonomyMotorAndSensorSetup();
        waitForStart();

        while(opModeIsActive()){
            KeepStraightUsingRangeSensors();
        }
    }
}
