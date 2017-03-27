package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="AutoOneBeaconBlue", group="Autonomous")
public class AutoOneBeaconBlue extends CommonFunctions{
    final String ColorDeterminantBlue = "BLUE";

    @Override
    public void runOpMode() throws InterruptedException {
        AutonomyMotorAndSensorSetup();
        startCamera();
        waitForStart();

        while(gyro.isCalibrating()){

        }

//        DriveBackwardWithEncoder(15, 0.2); //Keep in mind that "backwards" means a drive where the spinner is in front. This will change as we turn around and push beacons.
//
//        shootBalls(); //Shoot balls into center goal. Might want to review shooting speed and sleep time with launcher improvements
//        sleep(4000);
//        stopShootingBalls();
//
//
//        turnRight(110); //Again, forwards is in the direction of the button pusher, so we turn right. Adjust angle as needed.
//
//        DriveForwardWithEncoder(20, 0.5); //Forwards is correct again. Get close up to the line
        DriveForwardWithEncoder(10, 0.4);

        turnRight(30);

        DriveForwardWithEncoder(35, 0.4);

        InterceptLine(ColorDeterminantBlue); //Use the ODS to intercept the line.

        SnapBackToLine(ColorDeterminantBlue); //Intercept the line in between both ODS Sensors
        firstLineDone = true;

        ///////////////////////////////////////////////////////STOPPING POINT - Here we decide where to go -- should we use the gyro to drive straight or use the ODS to track? All of this depends on the angle that we have after the snap back.

        TrackLineInwards(); //Option 2

        RedOrBlue(); //Use the camera to detect color and assign it to our color string

        try {
            PushButton(ColorDeterminantBlue);
        } catch (IOException e) {
            e.printStackTrace();
        }

        shootBalls();

        sleep(3000);

        stopShootingBalls();

        stopCamera();
        /*
        DriveBackwardWithEncoder(DriveBackDistance, 0.5); //Need to find a solid drive back distance

        StopDriveMotorsAndWait(500);


        turnLeft(55); //Turn to face the next line was 75 //was 60

        DriveForwardWithEncoder(41, 0.3); //Drive towards it and get close
        // sleep(5000);

        InterceptLine(ColorDeterminantBlue); //Intercept the second line

        SnapBackToLine(ColorDeterminantBlue); //Snap back to it

        TrackLineInwards(); //Option 2
//
        RedOrBlue(); //Use the camera to detect color and assign it to our color string
//
        PushButton(ColorDeterminantBlue);

        stopCamera();

//
//        ///////////////////////////////////////////////////////////////////////SAME DECISION AS ABOVE///////////////////////////////////
//
//        /*
//        DriveForwardWithDistanceSensor(5); //Option 1 ----- Need to look at how the gyro keeps us straight
//        TrackLineInwards(); //Option 2
//        */
//
//        RedOrBlue(); //Use the camera to detect color and assign it to our color string
//
//        PushButton(AllianceColor);
//
//        stopCamera();
//        stop();


    }

}