package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;

@Autonomous(name = "Light Sensor Test", group = "Sensor")
@Disabled

public class TestLineTracking extends LinearOpMode {
    DcMotor Left;
    DcMotor Right;

    LightSensor RightLight;  // Hardware Device Object
    LightSensor LeftLight;

    double TurnPower = 0.2;

    boolean LedOn = true;
    double LightThreshold = 0.4;
    int RunTimeMsec = 100;

    @Override
    public void runOpMode() {
        RightLight = hardwareMap.lightSensor.get("InnerRightLight");
        LeftLight = hardwareMap.lightSensor.get("InnerLeftLight");

        Left = hardwareMap.dcMotor.get("Left");
        Right = hardwareMap.dcMotor.get("Right");

        Left.setDirection(DcMotor.Direction.REVERSE);
        Right.setDirection(DcMotor.Direction.FORWARD);

        // Set the LED state in the beginning.
        RightLight.enableLed(LedOn);
        LeftLight.enableLed(LedOn);

        // wait for the start button to be pressed.
        waitForStart();

        while (RightLight.getLightDetected() > LightThreshold){
            Left.setPower(0.2);
            Right.setPower(0.2);
        }

        Left.setPower(0);
        Right.setPower(0);

        // while the op mode is active, loop and read the light levels.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            if(RightLight.getLightDetected() < LightThreshold && LeftLight.getLightDetected() < LightThreshold){ // Both detect dark values
                int InitialClicks = Right.getCurrentPosition();
                telemetry.addLine("Both are dark");
                Right.setPower(TurnPower);
                Left.setPower(TurnPower);
                sleep(RunTimeMsec);

            } else if (RightLight.getLightDetected() > LightThreshold && LeftLight.getLightDetected() < LightThreshold){ //Right is dark, left is light
                telemetry.addLine("Turn Right");
                Right.setPower(TurnPower);
                Left.setPower(-TurnPower);
                sleep(RunTimeMsec);
            } else if (RightLight.getLightDetected() < LightThreshold && LeftLight.getLightDetected() > LightThreshold){ //Right is light and left is dark
                telemetry.addLine("Turn Left");
                Right.setPower(-TurnPower);
                Left.setPower(TurnPower);
                sleep(RunTimeMsec);
            } else if (RightLight.getLightDetected() > LightThreshold && LeftLight.getLightDetected() > LightThreshold){ //Both are light
                telemetry.addLine("Both Light");
                Right.setPower(TurnPower);
                Left.setPower(TurnPower);
                sleep(RunTimeMsec);
            } else {
                telemetry.addLine("WTF");
            }

            Right.setPower(0);
            Left.setPower(0);
            sleep(100);

            telemetry.addData("Normal Right", RightLight.getLightDetected());
            telemetry.addData("Normal Left", LeftLight.getLightDetected());

            telemetry.update();
        }
    }
}
