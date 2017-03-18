package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red", group="Pushbot")
public class AutoPrimaryRed extends CommonFunctions{

    final String AllianceColor = "RED";

    @Override
    public void runOpMode() throws InterruptedException {

        AutonomyMotorAndSensorSetup();

        waitForStart();

        while(gyro.isCalibrating()){

        }

//
//        DriveBackwardWithEncoder(15, 0.2); //Keep in mind that "backwards" means a drive where the spinner is in front. This will change as we turn around and push beacons.
//
//        shootBalls(); //Shoot balls into center goal. Might want to review shooting speed and sleep time with launcher improvements
//        sleep(4000);
//        stopShootingBalls();


//        turnRight(110); //Again, forwards is in the direction of the button pusher, so we turn right. Adjust angle as needed.

        DriveForwardWithEncoder(20, 0.5); //Forwards is correct again. Get close up to the line

  //      DriveForwardWithEncoder(10, 0.4);

        turnLeft(40);

        DriveForwardWithEncoder(35, 0.4);

        AlignWithLine(AllianceColor,0.17);
//
//        ///////////////////////////////////////////////////////STOPPING POINT - Here we decide where to go -- should we use the gyro to drive straight or use the ODS to track? All of this depends on the angle that we have after the snap back.
//
//        DriveForwardWithDistanceSensor(5); //Option 1 ----- Need to look at how the gyro keeps us straight
//        TrackLineInwards(); //Option 2
//
        RedOrBlue(); //Use the camera to detect color and assign it to our color string
//
        PushButton(AllianceColor);

        sleep(5000);

        shootBalls();

        sleep(3000); // was 3

        stopShootingBalls();

        gyro.resetZAxisIntegrator();

        DriveBackwardWithEncoder(DriveBackDistance, 0.4); //Need to find a solid drive back distance

        StopAndWait(500);


      //  DriveBackwardWithEncoder(10,0.5);

        turnRight(70); //Turn to face the next line was 75 // was 60 // was 55 // was50 // was 53

        DriveForwardWithEncoder(45, 0.4); //Drive towards it and get close
       // sleep(5000);

        AlignWithLine(AllianceColor,.17);

 //       TrackLineInwards(); //Option 2
//
        RedOrBlue(); //Use the camera to detect color and assign it to our color string
//
        PushButton(AllianceColor);

        stopCamera();


        ///////////////////////////////////////////////////////////////////////SAME DECISION AS ABOVE///////////////////////////////////

        /*
        DriveForwardWithDistanceSensor(5); //Option 1 ----- Need to look at how the gyro keeps us straight
        TrackLineInwards(); //Option 2
        */

        RedOrBlue(); //Use the camera to detect color and assign it to our color string

        PushButton(AllianceColor);

        stopCamera();
        stop();
    }

}