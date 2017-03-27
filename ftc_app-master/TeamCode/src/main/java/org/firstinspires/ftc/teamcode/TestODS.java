package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by RobotAdmin on 2/16/2017.
 */

@Autonomous(name = "Sensor TEST", group = "Sensor")
@Disabled
public class TestODS extends CommonFunctions{
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomyMotorAndSensorSetup();

        waitForStart();

        while(gyro.isCalibrating()){

        }

        Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()){
            telemetry.addData("Right ODS", InnerRightLight.getLightDetected());
            telemetry.addData("Left ODS", InnerLeftLight.getLightDetected());


            telemetry.update();
        }
    }
}
