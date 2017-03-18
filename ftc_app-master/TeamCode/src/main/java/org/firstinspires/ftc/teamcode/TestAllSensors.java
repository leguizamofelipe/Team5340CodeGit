package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="ALL Sensor Test", group="Pushbot")

public class TestAllSensors extends CommonFunctions{

    @Override
    public void runOpMode() throws InterruptedException {

        AutonomyMotorAndSensorSetup();

        waitForStart();

        Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DriveForwardWithEncoder(30, 0.3);

        while (opModeIsActive()) {
            telemetry.addData("Right", Right.getCurrentPosition());
            telemetry.addData("Left", Left.getCurrentPosition());
            telemetry.update();
        }
    }
}
