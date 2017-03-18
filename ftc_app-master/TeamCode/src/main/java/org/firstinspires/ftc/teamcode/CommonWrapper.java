package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import for_camera_opmodes.RunCamera;


public class CommonWrapper extends LinearOpMode {

    //////////////////////////////////////////////////////////////////////////////////////////////
    /////                               Instances of Useful Classes                           ////
    /////////////////////////////////////////////////////////////////////////////////////////////

    CommonDrivingFunctions Driving = new CommonDrivingFunctions();
    CommonMotorAndSensorSetup HardwareSetup = new CommonMotorAndSensorSetup();
    CommonShootingMechanismFunctions ShootingFunctions = new CommonShootingMechanismFunctions();
    CommonInteractionWithLineAndBeacons TrackingAndBeaconFunctions = new CommonInteractionWithLineAndBeacons();
    CommonVariables Variables = new CommonVariables();
    RunCamera RunCamera = new RunCamera();

    //////////////////////////////////////////////////////////////////////////////////////////////
    /////                               Hardware Setup Calls                                  ////
    /////////////////////////////////////////////////////////////////////////////////////////////

    public void AutonomyMotorAndSensorSetup(HardwareMap HWMAP) throws InterruptedException {
        Variables.AutonomyMotorAndSensorSetup(HWMAP);
    }

    public void TeleopMotorAndSensorSetup() throws InterruptedException {
        HardwareSetup.AutonomyMotorAndSensorSetup();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    /////                               Driving Calls                                        ////
    /////////////////////////////////////////////////////////////////////////////////////////////

    public void DriveForward(double Distance, double Speed) throws InterruptedException {
        Driving.DriveForwardWithEncoder(Distance, Speed);
    }

    public void DriveBackward(double Distance, double Speed) throws InterruptedException {
        Driving.DriveBackwardWithEncoder(Distance, Speed);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    /////                               Formulas                                           ////
    /////////////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////////////////
    /////                               ODS Light Readings (Booleans)                        ////
    /////////////////////////////////////////////////////////////////////////////////////////////

    public boolean InnerRightDetectsLight(){
        return TrackingAndBeaconFunctions.InnerRightDetectsLight();
    }

    public boolean InnerLeftDetectsLight(){
        return TrackingAndBeaconFunctions.InnerLeftDetectsLight();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    /////                               Line Tracking Calls                                  ////
    /////////////////////////////////////////////////////////////////////////////////////////////

    public void InterceptLine(String Determinant) throws InterruptedException {
        TrackingAndBeaconFunctions.InterceptLine(Determinant);
    }

    public void StopAndWait(int TimeToWait) {
        TrackingAndBeaconFunctions.StopAndWait(TimeToWait);
    }

    public void SnapBackToLine(String Determinant) throws InterruptedException {
        TrackingAndBeaconFunctions.SnapBackToLine(Determinant);
    }

    public void TrackLineInwards() throws InterruptedException {
        TrackingAndBeaconFunctions.TrackLineInwards();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    /////                               Camera And Button Pushing                             ////
    /////////////////////////////////////////////////////////////////////////////////////////////

    public String RedOrBlue() throws InterruptedException {
        return RunCamera.RedOrBlue();
    }

    public void PushButton(String ColorDeterminant) throws InterruptedException {
        TrackingAndBeaconFunctions.PushButton(ColorDeterminant);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    /////                               Shooter, Launcher Calls                               ////
    /////////////////////////////////////////////////////////////////////////////////////////////

    public void turnOnFlapper() throws InterruptedException {
        ShootingFunctions.turnOnFlapper();
    }

    public void TurnOnLaunchers() throws InterruptedException {
        ShootingFunctions.TurnOnLaunchers();
    }

    public void shootBalls() throws InterruptedException {
        ShootingFunctions.shootBalls();
    }

    public void stopShootingBalls() throws InterruptedException {
        ShootingFunctions.stopShootingBalls();
    }

    public void TurnOffLift() throws InterruptedException {
        ShootingFunctions.TurnOffLift();
    }

    public void TurnOffLaunchers() throws InterruptedException {
        ShootingFunctions.TurnOffLaunchers();
    }

    public void TurnOffFlapper() throws InterruptedException {
        ShootingFunctions.TurnOffFlapper();
    }

    public void TurnOnLift() throws InterruptedException {
        ShootingFunctions.TurnOnLift();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    /////                               Gyro Calls                                           ////
    /////////////////////////////////////////////////////////////////////////////////////////////


    public void turnRight(int DesiredDegree) throws InterruptedException {
        Driving.turnRight(DesiredDegree);
    }

    public void turnLeft(int DesiredDegree) throws InterruptedException{
        Driving.turnLeft(DesiredDegree);
    }

    public void runToDistanceAwayFromWall(double distanceToStop){
        Driving.DriveForwardWithDistanceSensor(distanceToStop);
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }


//    public int DegreesOff(int Target, int Current) throws InterruptedException{
//        if(Math.abs(Target - Current) < DegreesOffTolerance){
//            return 0;
//        } else{
//            return Math.abs(Target - Current) - DegreesOffTolerance;
//        }
//    }
//
//    public void PrintFile(String Line1, String Line2, String Line3){
//        try {
//            File TextFile = new File("/sdcard/FIRST/Log.txt");
//            FileOutputStream filestream = new FileOutputStream(TextFile);
//            PrintStream PrintStream = new PrintStream(filestream);
//
//            PrintStream.println(Line1);
//            PrintStream.println(Line2);
//            PrintStream.println(Line3);
//
//        } catch (FileNotFoundException e) {
//            e.printStackTrace();
//        }
//    }
//
//    public void RunBitmapPreview(){
//        ((FtcRobotControllerActivity)this.hardwareMap.appContext).StartBitmapPreview(SavedBitmap);
//    }
}

