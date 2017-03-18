package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by robotadmin on 2/6/2017.
 */
@Autonomous(name = "Gyro", group = "Concept")
@Disabled

public class TestGyro extends LinearOpMode{
    DcMotor Left;
    DcMotor Right;
    GyroSensor GyroScope;
    ModernRoboticsI2cGyro gyro;

    @Override
    public void runOpMode() throws InterruptedException {
        Left = hardwareMap.dcMotor.get("Left");
        Right = hardwareMap.dcMotor.get("Right");
        Right.setDirection(DcMotorSimple.Direction.REVERSE);

        GyroScope = hardwareMap.gyroSensor.get("Gyro");

        gyro = (ModernRoboticsI2cGyro) GyroScope;

        gyro.calibrate();

        int zAccumulated, heading, xVal,yVal,zVal;

        waitForStart();

        while(gyro.isCalibrating()){
        }

        while (opModeIsActive()){
            zVal = gyro.rawZ();
            xVal = gyro.rawX();
            yVal = gyro.rawY();
            heading = gyro.getHeading();
            zAccumulated = gyro.getIntegratedZValue();

//            while(zAccumulated > 0 || zAccumulated < 0 && opModeIsActive()) {
//                zAccumulated = gyro.getIntegratedZValue();
//                if (zAccumulated < 0) {
//                    double powerFix = Math.abs(zAccumulated);
//                    powerFix = powerFix / 100;
//                    if(powerFix < 0){
//                        powerFix = 0;
//                    }else if (powerFix > .85){
//                        powerFix = .85;
//                    }
//                    Right.setPower(.15 + powerFix );
//                    Left.setPower(.15 - powerFix);
//                } else if(zAccumulated > 0){
//                    double powerFix = Math.abs(zAccumulated);
//                    powerFix = powerFix / 100;
//                    if(powerFix < 0){
//                        powerFix = 0;
//                    }else if(powerFix > .85){
//                        powerFix = .85;
//                    }
//                    Right.setPower(.15 - powerFix);
//                    Left.setPower(.15 + powerFix);
//                }
//                telemetry.addData("Integrated Z Value ",zAccumulated);
//                telemetry.update();
//                waitOneFullHardwareCycle();
//            }
//
//            Right.setPower(.15);
//            Left.setPower(.15);
            telemetry.addData("Zvalue",zVal);
            telemetry.addData("xvalue",xVal);
            telemetry.addData("yvalue",yVal);
            telemetry.addData("heading",heading);
            telemetry.addData("Integrated Z Value ",zAccumulated);

            telemetry.update();
        }

    }
}
