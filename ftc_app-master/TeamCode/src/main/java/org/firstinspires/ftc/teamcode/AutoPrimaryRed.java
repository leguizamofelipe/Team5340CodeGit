package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.FileNotFoundException;
import java.io.IOException;

@Autonomous(name="Red", group="Pushbot")
public class AutoPrimaryRed extends CommonFunctions{

    final String AllianceColor = "RED";

    String FirstRun;
    String SecondRun;

    boolean RunLogger = true;

    @Override
    public void runOpMode() throws InterruptedException {
        ///////////LOGGER SETUP/////////////////////////
        try {
            Logger = new Logger(RunLogger, "RedAuto");
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        ///////////END LOGGER SETUP/////////////////////////

        AutonomyMotorAndSensorSetup(); //Setup all motors, sensors, servos

        startCamera(); //Open up camera, start preview on phone

        waitForStart();

        while(gyro.isCalibrating()){
        }

        DriveForwardWithEncoder(20, 0.5); //Drive towards center structure

        turnLeft(47); // was 45 // Turn and face the line

        DriveForwardWithEncoder(63, 0.5); //Drive to get close to the line

        AlignWithLineUsingODS(AllianceColor, 0.17); //Intercept line, align with it

        TrackLineInwards(); //Track the line until we are a good distance away from the beacon

        try {
            PushButton(AllianceColor); // Detect the color using the camera, push the beacon button
        } catch (IOException e) {
            e.printStackTrace();
        }

        DriveBackwardWithEncoder(5, 0.3); //Drive back in preparation for squaring up with the wall

        TurnOnLaunchers(); //Allow motors time for ramp up

        Blocker.setPosition(blockerUp);

        StopDriveMotorsAndWait(100); //Lose momentum

        while(!isSquare) {
            SquareUpWithWallUsingDistance(); //Get straight with wall
        }

        gyro.resetZAxisIntegrator();

        Blocker.setPosition(blockerDown);

        shootBalls();

        sleep(3000); //Can adjust time if needed

        stopShootingBalls();

        DriveBackwardWithEncoder(DriveBackDistance, 0.4); //Need to find a solid drive back distance

        StopDriveMotorsAndWait(200); //Lose momentum

        turnRight(75); //was 70

        DriveForwardWithEncoder(59, 0.7); // was 0.55, Drive towards the line and get close

        AlignWithLineUsingODS(AllianceColor,.17); //Intercept line, align with it

        TrackLineInwards();

        try {
            PushButton(AllianceColor);
        } catch (IOException e) {
            e.printStackTrace();
        }

        stopCamera();
    }

}