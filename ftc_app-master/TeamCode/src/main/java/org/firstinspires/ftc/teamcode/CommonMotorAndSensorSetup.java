package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


/**
 * Created by robotadmin on 3/10/2017.
 */

public class CommonMotorAndSensorSetup extends LinearOpMode{
    CommonVariables commonVariables = new CommonVariables();

    ///////////////////////////////////MOTOR AND SENSOR SETUP/////////////////////////////////////

    public void AutonomyMotorAndSensorSetup() throws InterruptedException {

        commonVariables.Left = hardwareMap.dcMotor.get("Left");  // Motor setup
        commonVariables.Right = hardwareMap.dcMotor.get("Right");
        commonVariables.Left.setDirection(DcMotor.Direction.REVERSE);
        commonVariables.Right.setDirection(DcMotor.Direction.FORWARD);

        commonVariables.Pusher = hardwareMap.servo.get("Pusher");

        commonVariables.Flapper = hardwareMap.dcMotor.get("Flapper");
        commonVariables.Launcher1 = hardwareMap.dcMotor.get("Launcher1");
        commonVariables.Launcher2 = hardwareMap.dcMotor.get("Launcher2");
        commonVariables.Lift = hardwareMap.dcMotor.get("Lift");
        commonVariables.Launcher1.setDirection(DcMotor.Direction.REVERSE);
        commonVariables.Launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        commonVariables.Launcher2.setDirection(DcMotor.Direction.FORWARD);
        commonVariables.Launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        commonVariables.Lift.setDirection(DcMotor.Direction.FORWARD);
        commonVariables.Flapper.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("Got Motors, Servos");
        telemetry.update();

        commonVariables.rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"range");

        commonVariables.InnerRightLight = hardwareMap.opticalDistanceSensor.get("RightLight"); // Sensor setup
        commonVariables.InnerLeftLight = hardwareMap.opticalDistanceSensor.get("LeftLight");

        ////////////////////////////GYRO///////////////////////////
        commonVariables.Gyro = hardwareMap.gyroSensor.get("Gyro");
        commonVariables.gyro = (ModernRoboticsI2cGyro) commonVariables.Gyro;

        commonVariables.gyro.calibrate();

        //////////////////////////////SetServo Posititon  ////////////////////////////
        commonVariables.Pusher.setPosition(commonVariables.NeutralPosition);

    }

    public void TeleopMotorAndSensorSetup() throws InterruptedException{
        commonVariables.Left = hardwareMap.dcMotor.get("Left");  //Motor setup
        commonVariables.Right = hardwareMap.dcMotor.get("Right");

        commonVariables.Flapper = hardwareMap.dcMotor.get("Flapper");
        commonVariables.Launcher1 = hardwareMap.dcMotor.get("Launcher1");
        commonVariables.Launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        commonVariables.Launcher2 = hardwareMap.dcMotor.get("Launcher2");
        commonVariables.Launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        commonVariables.Lift = hardwareMap.dcMotor.get("Lift");

        commonVariables.Pusher = hardwareMap.servo.get("Pusher");

        commonVariables.Left.setDirection(DcMotor.Direction.FORWARD);
        commonVariables.Right.setDirection(DcMotor.Direction.REVERSE);

        commonVariables.Launcher1.setDirection(DcMotor.Direction.REVERSE);
        commonVariables.Launcher2.setDirection(DcMotor.Direction.FORWARD);

        commonVariables.Lift.setDirection(DcMotor.Direction.FORWARD);
        commonVariables.Flapper.setDirection(DcMotorSimple.Direction.FORWARD);

        commonVariables.Pusher.setPosition(commonVariables.NeutralPosition);

    }

    public void runOpMode() throws InterruptedException {
    }
}
