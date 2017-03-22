package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.Logger;

import java.io.FileNotFoundException;
import java.io.IOException;

import for_camera_opmodes.RunCamera;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */

@TeleOp(name = "TestCameraRedOrBlue", group = "ZZOpModeCameraPackage")
//@Disabled
public class TestCameraRedOrBlue extends RunCamera {
    Logger CameraLogger;


    @Override
    public void runOpMode() {

        String colorString = "NONE";

        if (isCameraAvailable()) {

            try {
                CameraLogger = new Logger(true, "Test");
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            }

            setCameraDownsampling(8);

            startCamera();  // can take a while.

            waitForStart();

            boolean gotimage = false;

            colorString = RedOrBlue();

            while(opModeIsActive()){
                if(!gotimage){
                    try {
                        CameraLogger.saveImage(returnBitmap());
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                    gotimage = true;
                }
            }


        }
    }
}
