package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by robotadmin on 3/10/2017.
 */

public class CommonInteractionWithLineAndBeacons extends CommonMotorAndSensorSetup {
    //////////////////////////////////////////////////////////////////////////////////////////////
    /////                               CommonDrivingFunctions  Instance                     ////
    /////////////////////////////////////////////////////////////////////////////////////////////

    CommonDrivingFunctions drivingFunction = new CommonDrivingFunctions();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    //////                             Formulas and Booleans                                   /////
    ////////////////////////////////////////////////////////////////////////////////////////////////
    public void StopAndWait(int TimeToWait) {
        commonVariables.Left.setPower(0);
        commonVariables.Right.setPower(0);
        sleep(TimeToWait); // Stop and think
    }//end StopDriveMotorsAndWait

    boolean InnerRightDetectsLight() {
        if (commonVariables.InnerRightLight.getLightDetected() > commonVariables.ODSLightThreshold) {
            return true;
        } else {
            return false;
        }
    }//end RightDetectsLight

    boolean InnerLeftDetectsLight() {
        if (commonVariables.InnerLeftLight.getLightDetected() > commonVariables.ODSLightThreshold) {
            return true;
        } else {
            return false;
        }
    }//end LeftDetectsLight


    ////////////////////////////////////////////////////////////////////////////////////////////
    ////////         Actual Functions  that interact with line                            /////
    //////////////////////////////////////////////////////////////////////////////////////////

    public void InterceptLine(String Determinant) throws InterruptedException { //Intercept line for the first time
        if (Determinant.equals("RED")) {
            while (!InnerLeftDetectsLight()){// && !OuterLeftDetectsLight()) { // Initial drive -- needed to catch the white tape
                commonVariables.Left.setPower(commonVariables.PowerForInterceptingDrive); // Was -.18
                commonVariables.Right.setPower(commonVariables.PowerForInterceptingDrive);// Was -.18
                if (InnerLeftDetectsLight()){
                    commonVariables.InnerLeftSensorTriggered = true;
                }

            }
        } else if (Determinant.equals("BLUE")) {
            while (!InnerRightDetectsLight()) {// && !OuterRightDetectsLight()) { // Initial drive -- needed to catch the white tape
                commonVariables.Left.setPower(commonVariables.PowerForInterceptingDrive);
                commonVariables.Right.setPower(commonVariables.PowerForInterceptingDrive);
                if (InnerRightDetectsLight()){
                    commonVariables.InnerRightSensorTriggered = true;
                }

            }
        }

        StopAndWait(250);
    }

    public void SnapBackToLine(String Determinant) throws InterruptedException {
        if (Determinant.equals("RED")) {

            if (commonVariables.firstLineDone){
                drivingFunction.DriveForwardWithEncoder(2, 0.3);
            }

            while (!InnerRightDetectsLight()) {// && !OuterRightDetectsLight()) {

                telemetry.addLine("In !InnerRight");
                telemetry.update();
                if (InnerLeftDetectsLight()) {
                    commonVariables.InnerLeftSensorTriggered = true;
                }

                if (!commonVariables.InnerLeftSensorTriggered) {
                    commonVariables.Right.setPower(-commonVariables.FastSnapSpeed);
                    commonVariables.Left.setPower(commonVariables.FastSnapSpeed);
                    telemetry.addLine("Fast snap turn");
                    telemetry.update();
                } else if (commonVariables.InnerLeftSensorTriggered){
                    commonVariables.Right.setPower(-commonVariables.SlowSnapSpeed);
                    commonVariables.Left.setPower(commonVariables.SlowSnapSpeed);
                    telemetry.addLine("Slow Snap turn");
                    telemetry.update();
                }
            }
        } else if (Determinant.equals("BLUE")) {

            if (commonVariables.firstLineDone){
                drivingFunction.DriveForwardWithEncoder(1, 0.3);
            }

            while (!InnerLeftDetectsLight()) {// && !OuterLeftDetectsLight()) {

                if (InnerRightDetectsLight()){
                    commonVariables.InnerRightSensorTriggered = true;
                }
                if (!commonVariables.InnerRightSensorTriggered) {
                    commonVariables.Right.setPower(commonVariables.FastSnapSpeed);
                    commonVariables.Left.setPower(-commonVariables.FastSnapSpeed);
                } else {
                    commonVariables.Right.setPower(commonVariables.SlowSnapSpeed);
                    commonVariables.Left.setPower(-commonVariables.SlowSnapSpeed);
                }
            }
        }

        StopAndWait(250);
       // ResetTriggers();

    }

    //We assume the robot is relatively squared up on line, and that the line is captured between the ODS sensors
    public void TrackLineInwards() {
        double currentDistance = commonVariables.rangeSensor.getDistance(DistanceUnit.CM);

        while (currentDistance > commonVariables.StopDistanceFromWall && opModeIsActive()) {
            if (!InnerRightDetectsLight() && !InnerLeftDetectsLight() && opModeIsActive()) { // Both detect dark values, drive forward
                telemetry.addLine("Both are dark");
                commonVariables.Right.setPower(commonVariables.ForwardDrivingSpeed);
                commonVariables.Left.setPower(commonVariables.ForwardDrivingSpeed);
            } else if (InnerRightDetectsLight() && !InnerLeftDetectsLight() && opModeIsActive()) { //Right is light, left is dark, turn left
                telemetry.addLine("Turn Right");
                commonVariables.Right.setPower(commonVariables.TurningTrackSpeed);
                commonVariables.Left.setPower(-commonVariables.TurningTrackSpeed);
            } else if (!InnerRightDetectsLight() && InnerLeftDetectsLight() && opModeIsActive()) { //Right is light and left is dark
                telemetry.addLine("Turn Left");
                commonVariables.Right.setPower(-commonVariables.TurningTrackSpeed);
                commonVariables.Left.setPower(commonVariables.TurningTrackSpeed);
            } else if (InnerLeftDetectsLight() && InnerRightDetectsLight() && opModeIsActive()) { //Both are light
                telemetry.addLine("Both Light");
                commonVariables.Right.setPower(commonVariables.ForwardDrivingSpeed);
                commonVariables.Left.setPower(commonVariables.ForwardDrivingSpeed);
            } else {
                telemetry.addLine("Nothing!");
            }

            sleep(commonVariables.RunTimeMsec);

            commonVariables.Right.setPower(0);
            commonVariables.Left.setPower(0);
            currentDistance = commonVariables.rangeSensor.getDistance(DistanceUnit.CM);
        }
    }//end TrackLine Inwards Function

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////                                      Interactions  with Beacons                                                     /////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public void PushButton(String AllianceColor) throws InterruptedException {

        if(AllianceColor.equals(commonVariables.colorString)){
            commonVariables.Pusher.setPosition(commonVariables.PushRight);
        }else{
            commonVariables.Pusher.setPosition(commonVariables.PushLeft);
        }
        sleep(500);
        drivingFunction.DriveForwardWithEncoder(commonVariables.ButtonPushDriveDistance, commonVariables.PushingDriveSpeed);

        commonVariables.Pusher.setPosition(commonVariables.NeutralPosition);
        sleep(200);
    }

}//end CommonInteractions with LineAndBeacons
