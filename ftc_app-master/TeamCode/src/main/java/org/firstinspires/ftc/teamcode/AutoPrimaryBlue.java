package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue", group="Autonomous")
public class AutoPrimaryBlue extends LinearOpMode {
    final String ColorDeterminantBlue = "BLUE";

    CommonShootingMechanismFunctions CSMF = new CommonShootingMechanismFunctions();
    CommonVariables CV = new CommonVariables();
    CommonDrivingFunctions CDF = new CommonDrivingFunctions();
    CommonInteractionWithLineAndBeacons CIWLDAB = new CommonInteractionWithLineAndBeacons();
    CommonMotorAndSensorSetup CMASS = new CommonMotorAndSensorSetup();

    @Override
    public void runOpMode() throws InterruptedException {
//        CMASS.AutonomyMotorAndSensorSetup();
////        CIWLDAB.startCamera();
//        waitForStart();
//
//        while(gyro.isCalibrating()){
//
//        }

//        DriveBackwarvd(15, 0.2); //Keep in mind that "backwards" means a drive where the spinner is in front. This will change as we turn around and push beacons.
//
//        shootBalls(); //Shoot balls into center goal. Might want to review shooting speed and sleep time with launcher improvements
//        sleep(4000);
//        stopShootingBalls();
//
//
//        turnRight(110); //Again, forwards is in the direction of the button pusher, so we turn right. Adjust angle as needed.
//
//        DriveForwardWithEncoder(20, 0.5); //Forwards is correct again. Get close up to the line

        CDF.DriveForwardWithEncoder(10, 0.4);

        CDF.turnRight(25);

        CDF.DriveForwardWithEncoder(35, 0.4);

        CIWLDAB.InterceptLine(ColorDeterminantBlue); //Use the ODS to intercept the line.

        CIWLDAB.SnapBackToLine(ColorDeterminantBlue); //Intercept the line in between both ODS Sensors
        CIWLDAB.commonVariables.firstLineDone = true;

        ///////////////////////////////////////////////////////STOPPING POINT - Here we decide where to go -- should we use the gyro to drive straight or use the ODS to track? All of this depends on the angle that we have after the snap back.

        CIWLDAB.TrackLineInwards(); //Option 2

//        CIWLDAB.RedOrBlue(); //Use the camera to detect color and assign it to our color string

        CIWLDAB.PushButton(ColorDeterminantBlue);

        CSMF.shootBalls();

        sleep(3000);

        CSMF.stopShootingBalls();

        CDF.DriveBackwardWithEncoder(CV.DriveBackDistance, 0.5); //Need to find a solid drive back distance

        CIWLDAB.StopAndWait(500);


        CDF.turnLeft(60); //Turn to face the next line was 75 //was 60 // was 55

        CDF.DriveForwardWithEncoder(41, 0.3); //Drive towards it and get close
        // sleep(5000);

        CIWLDAB.InterceptLine(ColorDeterminantBlue); //Intercept the second line

        CIWLDAB.SnapBackToLine(ColorDeterminantBlue); //Snap back to it

        CIWLDAB.TrackLineInwards(); //Option 2
//
//        CIWLDAB.RedOrBlue(); //Use the camera to detect color and assign it to our color string
//
        CIWLDAB.PushButton(ColorDeterminantBlue);

//        CIWLDAB.stopCamera();

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