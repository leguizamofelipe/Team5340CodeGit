package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red", group="Pushbot")
public class AutoPrimaryRed extends CommonFunctions{

    final String AllianceColor = "RED";

    String FirstRun;
    String SecondRun;

    @Override
    public void runOpMode() throws InterruptedException {

        AutonomyMotorAndSensorSetup();

        startCamera();

        waitForStart();

        while(gyro.isCalibrating()){

        }

        DriveForwardWithEncoder(20, 0.5); //Forwards is correct again. Get close up to the line

        turnLeft(45); // was 40

        DriveForwardWithEncoder(45, 0.4);

        AlignWithLine(AllianceColor,0.17);

        TrackLineInwards();

       // FirstRun = RedOrBlue(); //Use the camera to detect color and assign it to our color string

        PushButton(AllianceColor);

        DriveBackwardWithEncoder(5, 0.3);

        StopAndWait(400);

        SquareUpWithWallUsingDistance();

        gyro.resetZAxisIntegrator();

        shootBalls();

        sleep(3000); // was 3

        stopShootingBalls();

        DriveBackwardWithEncoder(DriveBackDistance, 0.4); //Need to find a solid drive back distance

        StopAndWait(500);

      //  DriveBackwardWithEncoder(10,0.5);

        turnRight(75); //was 70

        DriveForwardWithEncoder(45, 0.4); //Drive towards it and get close
       // sleep(5000);

        AlignWithLine(AllianceColor,.17);

        TrackLineInwards();

        SecondRun = RedOrBlue();

        PushButton(AllianceColor);

        stopCamera();

//        RunBitmapPreview();

        telemetry.addLine(FirstRun);
        telemetry.addLine(SecondRun);
        telemetry.update();

        sleep(400000);
        
    }

}