package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by robotadmin on 3/16/2017.
 */

@Autonomous(name = "Align With Line Test", group = "Sensor")

public class TestAlignWithLine extends CommonFunctions {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomyMotorAndSensorSetup();

        waitForStart();

//        while (gyro.isCalibrating() && opModeIsActive()) {
//
//        }

        Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        while(opModeIsActive()){
//            telemetry.addData("COR ODS", CenterOfRotation.getLightDetected());
//            telemetry.addData("Alignment guide ODS", AlignmentGuide.getLightDetected());
//
//            telemetry.update();
//        }

//        AlignWithLine("RED", 0.17);
//
//        sleep(2000);

        StraightenUpWithLine(0.15);
    }

}
