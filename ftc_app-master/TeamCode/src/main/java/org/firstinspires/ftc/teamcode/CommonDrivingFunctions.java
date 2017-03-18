package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by robotadmin on 3/10/2017.
 */

public class CommonDrivingFunctions extends CommonVariables {

    ///////////////////////////////////////////////////////////////////////////////////////////////
    /////                                     Formulas                                         ////
    //////////////////////////////////////////////////////////////////////////////////////////////
    private int DistanceToClicksConverter(double DistanceToTravel) {

        double DistancePerRotation = WheelDiameter * Math.PI;
        double RotationsNeeded = (-DistanceToTravel / DistancePerRotation);
        int ClicksNeeded = (int) (RotationsNeeded * ClicksPerRotation * GearRatio);

        return ClicksNeeded;
    }//end Distatnce to clicks Converter

    /////////////////////////////////////////Find If Within Gyro Tolerance////////////////////////
    private int DegreesFromTolerance(int Target, int Current){
        if(Math.abs(Target - Current) < DegreesOffTolerance){
            return 0;
        } else{
            return Math.abs(Target - Current) - DegreesOffTolerance;
        }
    }///end DegreesFromTolerance formula



    ///////////////////////////////////////////////////////////////////////////////////////////////
    ////                            Drive Forward using gyro / encoder Only                         ////
    //////////////////////////////////////////////////////////////////////////////////////////////

    public void DriveForwardWithEncoder(double DistanceToTravel, double Speed) throws InterruptedException {
        int degreeToStayAt = gyro.getIntegratedZValue();

        //clear any previous encoder targets
        Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int ClicksNeeded = DistanceToClicksConverter(DistanceToTravel);

        Left.setTargetPosition(ClicksNeeded);
        Right.setTargetPosition(ClicksNeeded);

        Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Left.setPower(Speed);
        Right.setPower(Speed);


        while (opModeIsActive() && Right.isBusy() && Left.isBusy()) {
            DriveStraightWithGyro(degreeToStayAt, Speed);
        }

        Left.setPower(0);
        Right.setPower(0);

        //reset mode as a safety
        Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////                                   For Driving Backwards using gyro                                              ////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public void DriveBackwardWithEncoder(double DistanceToTravel, double Speed) throws InterruptedException {
        DriveForwardWithEncoder(-DistanceToTravel, -Speed);
    }//end DriveBackwardWithEncoder function

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////                                Driving Forward using Range Sensor / Gryo                                    ////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public void DriveForwardWithDistanceSensor(double distanceToStop){
        double currentDistance = rangeSensor.getDistance(DistanceUnit.CM);;
        int targetAngle = gyro.getIntegratedZValue();

        while(currentDistance > distanceToStop){
            if((currentDistance - distanceToStop) > slowDownDistance){
                DriveStraightWithGyro(targetAngle, FAST_POWER);
            }else{
                DriveStraightWithGyro(targetAngle, SLOW_POWER);
            }
            currentDistance = rangeSensor.getDistance(DistanceUnit.CM);
        }//end while

        Left.setPower(0);
        Right.setPower(0);

        Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }//end DriveForwardWithDistanceSensor

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///                                          Turning using Gyro                                            ///
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////


    ///If Turning Direction is 1 it means tur to the right and if turning direction is -1 it means turn left//////////////
    private void turn(int desiredAngle, int TurningDirection) throws InterruptedException { //Turning Direction = 1 when turning Right and turning Direction = -1 when turning Left
        gyro.resetZAxisIntegrator(); // resets to zero so the robots heading is relative to where we start are turn

        int currentDegree = Math.abs(gyro.getIntegratedZValue());
        int initialDegree = Math.abs(gyro.getIntegratedZValue());

        while (currentDegree < Math.abs(initialDegree + desiredAngle)) {
            telemetry.addData("ZVALUE:", Math.abs(gyro.getIntegratedZValue()));
            double power = 0.0;

            if(currentDegree < (Math.abs(initialDegree + desiredAngle) - 35)){ //Was at 25
                power = 0.3; //Was at .7
            } else {
                power = 0.15;//Was at .08
            }

            Right.setPower(power * TurningDirection);
            Left.setPower(-power * TurningDirection);
            telemetry.update();
            currentDegree = Math.abs(gyro.getIntegratedZValue());
        }//while

        Right.setPower(0);
        Left.setPower(0);
    }//end turn funciton

    /////////////////////////////Turn Right, the 1 means turn to the right/////////////////////////////////////////////
    public void turnRight(int DesiredDegree) throws InterruptedException{
        turn(DesiredDegree, 1);
    }
    ////////////////////////////Turn Left, the -1 means turn to the left//////////////////////////////////////////////
    public void turnLeft(int DesiredDegree) throws InterruptedException{
        turn(DesiredDegree,-1);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////                                   When Driving forwar Makes sure we dont veer ////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public void DriveStraightWithGyro(int degreeToStayAt, double Speed){
        int currentDegree = gyro.getIntegratedZValue();
        double powerAdjust = 0;

        if(currentDegree < degreeToStayAt && (DegreesFromTolerance(degreeToStayAt, currentDegree) > 0)){ //veering left
            powerAdjust = powerFactor * Math.abs(degreeToStayAt - currentDegree);
            if (powerAdjust > PowerAdjustCeiling){
                powerAdjust = PowerAdjustCeiling;
            }
            Right.setPower(Speed - powerAdjust);
            Left.setPower(Speed + powerAdjust);
        } else if(currentDegree > degreeToStayAt && (DegreesFromTolerance(degreeToStayAt, currentDegree) > 0)){ //veering right
            powerAdjust = powerFactor * Math.abs(degreeToStayAt - currentDegree);
            if (powerAdjust > PowerAdjustCeiling){
                powerAdjust = PowerAdjustCeiling;
            }
            Right.setPower(Speed + powerAdjust);
            Left.setPower(Speed - powerAdjust);
        }else{
            Right.setPower(Speed);
            Left.setPower(Speed);
        }
    }


}//End Common Driving Functions
