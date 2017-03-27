package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.FileNotFoundException;

@Autonomous(name = "Test Square Up With Wall", group = "Tests")

public class TestSquareUpWithWall extends CommonFunctions {
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            Logger = new Logger(true,"TWORANGESENSORTEST");
        }catch (FileNotFoundException e) {

            e.printStackTrace();
        }

        AutonomyMotorAndSensorSetup();

        waitForStart();

        while(!isSquare && opModeIsActive()){
            SquareUpWithWallUsingDistance();
        }
    }
}
