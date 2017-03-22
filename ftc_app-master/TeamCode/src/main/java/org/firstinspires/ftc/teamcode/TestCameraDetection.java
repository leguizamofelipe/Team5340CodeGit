package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import for_camera_opmodes.RunCamera;

@TeleOp
@Disabled
public class TestCameraDetection extends RunCamera{

    @Override
    public void runOpMode() throws InterruptedException {
        startCamera();

        waitForStart();

        RedOrBlue();

        sleep(4000);

        stopCamera();

//        RunBitmapPreview();

        sleep(40000);
    }
}
