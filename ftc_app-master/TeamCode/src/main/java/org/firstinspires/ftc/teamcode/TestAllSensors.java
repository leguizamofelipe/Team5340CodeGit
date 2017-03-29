package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="TestAllSensors", group="Tests")

public class TestAllSensors extends CommonFunctions{

    @Override
    public void runOpMode() throws InterruptedException {

        AutonomyMotorAndSensorSetup();

        waitForStart();

        Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            telemetry.addData("Right Motor Encoder", Right.getCurrentPosition());
            telemetry.addData("Left Motor Encoder", Left.getCurrentPosition());
            telemetry.addData("Gyro", gyro.getIntegratedZValue());
            telemetry.addData("Left ODS", LeftLight.getLightDetected());
            telemetry.addData("Right ODS", RightLight.getLightDetected());
            telemetry.addData("COR ODS", CenterOfRotation.getLightDetected());
            telemetry.addData("Alignment guide ODS", AlignmentGuide.getLightDetected());
            telemetry.addData("Right Distance", rightDistanceSensorReader.read(0x04, 2)[0] & 0xFF);
            telemetry.addData("Left Distance", leftDistanceSensorReader.read(0x04, 2)[0] & 0xFF);
            telemetry.update();
        }
    }
}
