package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "RangeValues with gyro  ", group = "Concept")
@Disabled
public class TestRangeSensor extends LinearOpMode {
    ModernRoboticsI2cRangeSensor rangeSensor;
    GyroSensor gyroScope;
    ModernRoboticsI2cGyro gyro;
    DcMotor Left;
    DcMotor Right;
    final double FAST_POWER = -0.4;
    double power = FAST_POWER;
    final double SLOW_POWER = -0.2;
    final double CEILING_FOR_CORRECTION = -.4;

    final int DEGREES_OFF_TOLERANCE = 4;
    double currentDistance;
    int currentAngle;
    int startingAngle;
    double slowDownDistance = 20;
    //double additionSpeedForTurnCorrection = -0.1;

    @Override
    public void runOpMode() throws InterruptedException {

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"range");
        gyroScope = hardwareMap.gyroSensor.get("gyro");
        gyro = (ModernRoboticsI2cGyro) gyroScope;


        Left = hardwareMap.dcMotor.get("Left");
        Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right = hardwareMap.dcMotor.get("Right");
        Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right.setDirection(DcMotorSimple.Direction.REVERSE);



        gyro.calibrate();

        waitForStart();

        while(gyro.isCalibrating()){
        }
        runToDistanceAwayFromWall(15.00);


//

//        while(currentDistance > stopDistance && opModeIsActive()){
//            if((currentDistance - stopDistance) <= 40){
//                power = -0.2;
//            }//end if
//
//            Right.setPower(power);
//            Left.setPower(power);

//            currentDistance = leftDistanceSensor.getDistance(DistanceUnit.CM);
//            telemetry.addData("CM AWAY", currentDistance);
//            telemetry.update();
//        }//end while
    }//end opMode

    public void runToDistanceAwayFromWall(double distanceToStop) throws InterruptedException{
        currentDistance = rangeSensor.getDistance(DistanceUnit.CM);
        startingAngle = gyro.getIntegratedZValue();


        while(currentDistance > distanceToStop){
            currentAngle = gyro.getIntegratedZValue();

            double additionSpeedForTurnCorrection  = -(double) (Math.abs(currentAngle - startingAngle) / 100);
            if(additionSpeedForTurnCorrection < CEILING_FOR_CORRECTION){
                additionSpeedForTurnCorrection = CEILING_FOR_CORRECTION;
            }


            if((currentDistance - distanceToStop) > slowDownDistance){
                power = FAST_POWER;
            }else{
                power = SLOW_POWER;
            }

            if(currentAngle < startingAngle && DegreesOff(startingAngle,currentAngle) != 0){
                Right.setPower(-(power + additionSpeedForTurnCorrection));
                Left.setPower((power + additionSpeedForTurnCorrection));
            }else if(currentAngle > startingAngle && DegreesOff(startingAngle,currentAngle) != 0){
                Right.setPower((power + additionSpeedForTurnCorrection));
                Left.setPower(-(power + additionSpeedForTurnCorrection));
            }else{
                Right.setPower(power);
                Left.setPower(power);
            }//end else

            currentDistance = rangeSensor.getDistance(DistanceUnit.CM);

            telemetry.addData("Current Distance", currentDistance);
            telemetry.addData("Stopping Distance", distanceToStop);
            telemetry.addData("Current Degree", currentAngle);
            telemetry.addData("StartingAngle", startingAngle);
            telemetry.addData("Power for wheels", power);
            telemetry.update();

        }//end while

    }//end DriveForwardWithDistanceSensor

    public int DegreesOff(int Target, int Current){
        if(Math.abs(Target - Current) < DEGREES_OFF_TOLERANCE){
            return 0;
        } else{
            return Math.abs(Target - Current) - DEGREES_OFF_TOLERANCE;
        }
    }//end public degreesOff


}//end class
