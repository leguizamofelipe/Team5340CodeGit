package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="TestEncoders", group="Tests")
@Disabled

public class TestEncoders extends LinearOpMode{
    double ClicksPerRotation = 280;
    double GearRatio = 3;
    double WheelDiameter = 4;
    double RobotDiameter = 13.75;

    DcMotor Left;
    DcMotor Right;

    public void DriveForward (double DistanceToTravel, double Speed){
        Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double DistancePerRotation = WheelDiameter * Math.PI;
        double RotationsNeeded = (DistanceToTravel/DistancePerRotation);
        int ClicksNeeded = (int)(RotationsNeeded * ClicksPerRotation * GearRatio);

        Left.setTargetPosition(ClicksNeeded);
        Right.setTargetPosition(ClicksNeeded);

        Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Left.setPower(Speed);
        Right.setPower(Speed);

        while (opModeIsActive() && Right.isBusy() && Left.isBusy()){
            idle();
            telemetry.addLine("Running with encoders to position");
            telemetry.addData("Right Encoder Position", Right.getCurrentPosition());
            telemetry.addData("Left Encoder Position", Left.getCurrentPosition());
            telemetry.addData("Target Clicks", ClicksNeeded);
            telemetry.update();
        }

        Left.setPower(0);
        Right.setPower(0);

        Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void TurnRight(double degrees, double speed){
        Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double distancePerRotation = WheelDiameter * Math.PI;
        double degreeRatio = degrees / 360;
        double distanceTravel = degreeRatio * (RobotDiameter * Math.PI);
        double RotationsNeeded = (distanceTravel/distancePerRotation);
        int ClicksNeeded = (int)(RotationsNeeded * ClicksPerRotation * GearRatio);

        Left.setTargetPosition(-ClicksNeeded);
        Right.setTargetPosition(ClicksNeeded);

        Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Left.setPower(-speed);
        Right.setPower(speed);

        while (opModeIsActive() && Right.isBusy() && Left.isBusy()){
            telemetry.addLine("Running with encoders to position");
            telemetry.addData("Right Encoder Position", Right.getCurrentPosition());
            telemetry.addData("Left Encoder Position", Left.getCurrentPosition());
        }

        Left.setPower(0);
        Right.setPower(0);

        Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void TurnLeft(double degrees, double speed){
        Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double distancePerRotation = WheelDiameter * Math.PI;
        double degreeRatio = degrees / 360;
        double distanceTravel = degreeRatio * (RobotDiameter * Math.PI);
        double RotationsNeeded = (distanceTravel/distancePerRotation);
        int ClicksNeeded = (int)(RotationsNeeded * ClicksPerRotation * GearRatio);


        Left.setTargetPosition(ClicksNeeded);
        Right.setTargetPosition(-ClicksNeeded);

        Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Left.setPower(speed);
        Right.setPower(-speed);

        while (opModeIsActive() && Right.isBusy() && Left.isBusy()){
            telemetry.addLine("Running with encoders to position");
            telemetry.addData("Right Encoder Position", Right.getCurrentPosition());
            telemetry.addData("Left Encoder Position", Left.getCurrentPosition());
        }

        Left.setPower(0);
        Right.setPower(0);

        Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Left = hardwareMap.dcMotor.get("Left");
        Right = hardwareMap.dcMotor.get("Right");

        Left.setDirection(DcMotor.Direction.FORWARD);
        Right.setDirection(DcMotor.Direction.REVERSE);
//
//        Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        DriveForward(10, 0.5);
        sleep(500);
        TurnRight(45, 0.1);
        sleep(500);

    }
}
