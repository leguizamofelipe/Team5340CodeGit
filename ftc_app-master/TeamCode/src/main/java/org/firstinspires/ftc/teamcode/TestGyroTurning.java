package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;

import java.lang.annotation.Target;

/**
 * Created by robotadmin on 2/6/2017.
 */
@Autonomous(name = "GyroTurningTest", group = "Concept")
@Disabled
public class TestGyroTurning extends LinearOpMode {
    DcMotor Left;
    DcMotor Right;
    GyroSensor GyroScope;
    ModernRoboticsI2cGyro gyro;

    //int zAccumulated;
    int currentDegree;
    double powerFactor = 0.05;
    double gearRatio = 3;
    double encoderClicks = 1478.4;
    double wheelDiameter = 3.5;
    int correctionFactor = 0; //
    int DegreesOffTolerance = 3;

    double PowerAdjustCeiling = 0.2;


    @Override
    public void runOpMode() throws InterruptedException {
        Left = hardwareMap.dcMotor.get("Left");
        Right = hardwareMap.dcMotor.get("Right");
        Right.setDirection(DcMotorSimple.Direction.REVERSE);
        Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        GyroScope = hardwareMap.gyroSensor.get("Gyro");

        gyro = (ModernRoboticsI2cGyro) GyroScope;

        gyro.calibrate();

     //   int zAccumulated, heading, xVal,yVal,zVal;

        waitForStart();

        while(gyro.isCalibrating()){
        }

        //while(opModeIsActive()) {
//            zVal = gyro.rawZ();
//            xVal = gyro.rawX();
//            yVal = gyro.rawY();
//            heading = gyro.getHeading();
//            zAccumulated = -gyro.getIntegratedZValue();
//
//            telemetry.addData("Zvalue",zVal);
//            telemetry.addData("xvalue",xVal);
//            telemetry.addData("yvalue",yVal);
//            telemetry.addData("heading",heading);
//            telemetry.addData("Integrated Z Value ",zAccumulated);
//            telemetry.update()
//        driveForward(100,.5);
          //  turnLeft(90);
          //  sleep(5000);
//        driveForward(30, 0.3);
//            sleep(1000);

//        turnRight(90);
//        sleep(1000);
//        turnRight(90);
//        sleep(1000);
//        turnRight(180);

//        driveForward(-500, -0.5);


        while(opModeIsActive()){
            telemetry.addData("ZVALUE:", gyro.getIntegratedZValue());
            telemetry.update();
        }
        //}
    }

    public void turnRight(int desiredAngle) throws InterruptedException{

        //int currentTurnCorrectionFactor =  (int)((Degrees / 90.00) * correctionFactor);

        currentDegree = -gyro.getIntegratedZValue();
        int initialDegree = -gyro.getIntegratedZValue();

        while (currentDegree < Math.abs(initialDegree - desiredAngle)) { // - Degrees + currentTurnCorrectionFactor
            currentDegree = -gyro.getIntegratedZValue();
            telemetry.addData("ZVALUE:", gyro.getIntegratedZValue());
            double power = 0.0;

            if(currentDegree < (Math.abs(initialDegree - desiredAngle) - 35)){
                power = 0.5;
            } else {
                power = 0.08;// works decent, 0.05 works very well but might burn motors
            }

            Right.setPower(-power);
            Left.setPower(power);
          //  telemetry.addData("Correction factor = ",currentTurnCorrectionFactor);
            telemetry.update();
            currentDegree = -gyro.getIntegratedZValue();
        }//while
        Right.setPower(0);
        Left.setPower(0);
    }//end right turn

    public void driveForward(double inches, double speed) throws InterruptedException{
        double powerAdjust = 0;

        currentDegree = gyro.getIntegratedZValue();
        int degreeToStayAt = gyro.getIntegratedZValue();

        Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = wheelDiameter * Math.PI;

        double amountOfClicksNeeded = ((inches / circumference) * gearRatio * encoderClicks);

        Right.setTargetPosition((int) amountOfClicksNeeded);
        Left.setTargetPosition((int) amountOfClicksNeeded);

        Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(Right.isBusy() && Left.isBusy()){

            currentDegree = gyro.getIntegratedZValue();
            if(currentDegree < degreeToStayAt && (DegreesOff(degreeToStayAt, currentDegree) > 0)){
                powerAdjust = powerFactor * Math.abs(degreeToStayAt - currentDegree); //(DegreesOff(degreeToStayAt,currentDegree));
                if (powerAdjust > PowerAdjustCeiling){
                    powerAdjust = PowerAdjustCeiling;
                }
                currentDegree = gyro.getIntegratedZValue();

                Right.setPower(speed + powerAdjust);
                Left.setPower(speed - powerAdjust);
            }else if(currentDegree > degreeToStayAt && (DegreesOff(degreeToStayAt, currentDegree) > 0)){
                powerAdjust = powerFactor * Math.abs(degreeToStayAt - currentDegree); //(DegreesOff(degreeToStayAt,currentDegree));
                if (powerAdjust > PowerAdjustCeiling){
                    powerAdjust = PowerAdjustCeiling;
                }
                currentDegree = gyro.getIntegratedZValue();

                Right.setPower(speed - powerAdjust);
                Left.setPower(speed + powerAdjust);
            }else{
                powerAdjust = 0;
                Right.setPower(speed);
                Left.setPower(speed);

            }

            telemetry.addData("Power adjust", powerAdjust);
            telemetry.addData("Current Degree", currentDegree);
            telemetry.addData("Right encoder count",Right.getCurrentPosition());
            telemetry.update();
        }//end while

        Right.setPower(0);
        Left.setPower(0);

        if(currentDegree < degreeToStayAt && (DegreesOff(degreeToStayAt, currentDegree) > 0)){
            powerAdjust = powerFactor * Math.abs(degreeToStayAt - currentDegree); //(DegreesOff(degreeToStayAt,currentDegree));
            if (powerAdjust > PowerAdjustCeiling){
                powerAdjust = PowerAdjustCeiling;
            }
            currentDegree = gyro.getIntegratedZValue();


            Right.setPower(speed + powerAdjust);
            Left.setPower(speed - powerAdjust);
        }else if(currentDegree > degreeToStayAt && (DegreesOff(degreeToStayAt, currentDegree) > 0)){
            powerAdjust = powerFactor * Math.abs(degreeToStayAt - currentDegree); //(DegreesOff(degreeToStayAt,currentDegree));
            if (powerAdjust > PowerAdjustCeiling){
                powerAdjust = PowerAdjustCeiling;
            }
            currentDegree = gyro.getIntegratedZValue();

            Right.setPower(speed - powerAdjust);
            Left.setPower(speed + powerAdjust);
        }else{
            powerAdjust = 0;
            Right.setPower(speed);
            Left.setPower(speed);

        }

    }//end driveForward


    public void turnLeft (int Degrees) throws InterruptedException{
        int currentTurnCorrectionFactor =  (int)((Degrees / 90.00) * correctionFactor);
        currentDegree = gyro.getIntegratedZValue();
        int valueInBeginning = gyro.getIntegratedZValue();
        while (currentDegree < (valueInBeginning + Degrees - currentTurnCorrectionFactor)) {
            telemetry.addData("ZVALUE", gyro.getIntegratedZValue());
            double power = (Degrees + Math.abs(valueInBeginning) - Math.abs(currentDegree))/100.00;
            if(power > .5){
                power = .5;
            }else if(power < .2){
                power = .2;
            }
            Right.setPower(power);
            Left.setPower(-power);
            currentDegree = gyro.getIntegratedZValue();
        }//end while statement
        Right.setPower(0);
        Left.setPower(0);
    }//end TurnLeftf

    public int DegreesOff(int Target, int Current){
        if(Math.abs(Target - Current) < DegreesOffTolerance){
            return 0;
        } else{
            return Math.abs(Target - Current) - DegreesOffTolerance;
        }
    }
}
