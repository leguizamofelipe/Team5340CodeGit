package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.FileNotFoundException;

@Autonomous(name = "Two Range Sensor Test", group = "Tests")

public class TestTwoRangeSensors extends CommonFunctions {
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            Logger = new Logger(true,"Two Range Sensor Test");
        }catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        AutonomyMotorAndSensorSetup();

        waitForStart();

        while(opModeIsActive()){
            int rightValue = ReadRangeSensorAndFilterValues(5, rightDistanceSensorReader);
                            //rightDistanceSensorReader.read(0x04, 2)[0] & 0xFF;
//            sleep(250); //was 250
            int leftValue = ReadRangeSensorAndFilterValues(5, leftDistanceSensorReader);
                            //leftDistanceSensorReader.read(0x04,2)[0] & 0xFF;

            Logger.printMessage("Right Distance", String.valueOf(rightValue));
            Logger.printMessage("Left Distance", String.valueOf(leftValue));

            telemetry.addData("Right Distance", String.valueOf(rightValue));
            telemetry.addData("Left Distance", String.valueOf(leftValue));
            telemetry.update();
        }
    }
}
