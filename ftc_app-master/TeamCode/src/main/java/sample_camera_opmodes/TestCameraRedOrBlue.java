package sample_camera_opmodes;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import for_camera_opmodes.RunCamera;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */

@TeleOp(name = "TestCameraRedOrBlue", group = "ZZOpModeCameraPackage")
//@Disabled
public class TestCameraRedOrBlue extends RunCamera {

    DcMotor motorRight;
    DcMotor motorLeft;

    int ds2 = 2;  // additional downsampling of the image
    // set to 1 to disable further downsampling

    @Override
    public void runOpMode() {

        String colorString = "NONE";

        // linear OpMode, so could do stuff like this too.
        /*
        motorLeft = hardwareMap.dcMotor.get("motor_1");
        motorRight = hardwareMap.dcMotor.get("motor_2");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        */

        if (isCameraAvailable()) {

            setCameraDownsampling(8);

            telemetry.addLine("Wait for camera to finish initializing!");
            telemetry.update();
            startCamera();  // can take a while.
            // best started before waitForStart
            telemetry.addLine("Camera ready!");
            telemetry.update();

            waitForStart();

            boolean gotimage = false;

            while (opModeIsActive()) {
                if(!gotimage){
                    returnBitmap().compress(Bitmap.CompressFormat.JPEG, 100, );
//
                }

                telemetry.addData("Color:", "Color detected is: " + RedOrBlue());
                telemetry.update();

            }
            stopCamera();

        }
    }
}
