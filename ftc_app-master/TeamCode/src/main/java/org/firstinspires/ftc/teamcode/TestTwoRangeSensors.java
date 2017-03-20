package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by RobotAdmin on 3/18/2017.
 */
@Autonomous(name = "Two Range Sensor Test", group = "Sensor")
public class TestTwoRangeSensors extends CommonFunctions {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomyMotorAndSensorSetup();

        waitForStart();

        while(!isSquare && opModeIsActive()){
            SquareUpWithWallUsingDistance();
        }
    }
}
