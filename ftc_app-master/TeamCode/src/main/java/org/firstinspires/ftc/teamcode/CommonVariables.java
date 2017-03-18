package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * Created by robotadmin on 3/10/2017.
 * TODO: add description for class
 *       add desc fro variable or improve commonVariables name
 *       make variables final/statics
 */

public class CommonVariables extends LinearOpMode{
    //////////////////////////////////////CAMERA //////////////////////////////////////

    Bitmap SavedBitmap = null; //Gets Bit map Values for logger
    String colorString; // uses this to hold the value of the string for color of beacon

    ////////////////////////////////////////SERVO VALUES////////////////////////////////////

    final double PushRight = 1.0; // Servo position to push button on rightside of beacon
    final double PushLeft = 0.0; // Servo Position to push left side of beacon
    final double NeutralPosition = 0.57; // Servo Position straight down

    //////////////////////////////////////DRIVING GLOBALS//////////////////////////////////////
    final double SlowSnapSpeed = 0.2; // was .2 // Turning speed to come back to line. After TRIPPING first ODS
    final double FastSnapSpeed = 0.3; // turning speed for allignment with line. Before TRIPPING first ODS
    final double PowerForInterceptingDrive = 0.22; // Drive forward speed until ods tripped on line
    final double TurnSpeed = 0.22; // Speed to turn robot with
    final double TurningTrackSpeed = 0.2; //Speed to use on line when getting closer to beacon
    final double ForwardDrivingSpeed = 0.2; //Used for forward drive on tracking, used .25
    final double PushingDriveSpeed = 0.5; //was 0.8 // Goes forward  to hit the beacon at this speed
    final double powerFactor = 0.05; // To account for gyro inaccuracy we use this to slow down motors closer we get to target degrees

    final double RightTurnSlipFactor = 1.4; //Encoder variables when we turned with encoders
    final double LeftTurnSlipFactor = 1.13;// when we turned with encoders
    final double ClicksPerRotation = 280; // for endcoders to do one full round
    final double GearRatio = 3;
    final double WheelDiameter = 4;
    final double RobotDiameter = 13.75;

    final double DistanceTraveled = 0;

    final int RunTimeMsec = 150; //was 100 // How Long should we wait after tracking the line upward.

    final double LiftPower = .75; // sets the speed of the lift to bring up balls

    final int DriveBackDistance = 15; //was 5 // after hitting first beacon how far we should back up

    final int ButtonPushDriveDistance = 12; //was 8 // drives forward set amount of distance to ensure we have enough time on pushing beacons
    final int StopDistanceFromWall = 16; // when tracking the line how far should we be before getting color values of beacon

    final double FAST_POWER = 0.4; // Fast power for robot to go (uses distance sensor) if above slow down distance
    final double SLOW_POWER = 0.2; // Slow power for robot to go (uses distance sensor) if below slow donwn distance
    final double CEILING_FOR_CORRECTION = .4;

    final double slowDownDistance = 20; // Tells robot you are 20 cm from target distance from wall and slows down the wheels

    boolean firstLineDone = false;// Are we done with the first beacon or not

    final double LauncherPowerForAuto = 0.22; // sets the power for the shooting mechanism in autonoumous

    final int DegreesOffTolerance = 4; //Tolerance for gyro to be within to be considered straight.

    /////////////////////////////////////VUFORIA///////////////////////////////////////////////
    VuforiaLocalizer vuforiaLocalizer;
    int AcceptableDistanceFromCenter = 175;
    int RedKillValue = 125;
    int BlueKillValue = 125;
    int StopDistance = -125;
    int CenterValue = 0;
    VuforiaTrackables beacons = null;
    boolean VuforiaVerification = false;
    int ds2 = 2;


    //Declares all the motors and sensors and servos we are using
    ///////////////////////////////////HARDWARE DECLARATIONS//////////////////////////////////
    static public DcMotor Left = null;
    static public DcMotor Right = null;

    static public DcMotor Flapper = null;
    static public DcMotor Launcher1 = null;
    static public DcMotor Launcher2 = null;
    static public DcMotor Lift = null;

    static OpticalDistanceSensor InnerRightLight;
    static OpticalDistanceSensor InnerLeftLight;

    static GyroSensor Gyro;
    static ModernRoboticsI2cGyro gyro;

    static ModernRoboticsI2cRangeSensor rangeSensor;

    static Servo Pusher;


    ///////////////////////////////////SENSOR VALUES/////////////////////////////////////////
    static boolean LedOn = true; // So the ODS can use their led lights on the sensor

    static boolean InnerLeftSensorTriggered = false; // determines if the left ods sensor trigger is triggered or not
    static boolean InnerRightSensorTriggered = false; // determines if the Right ODs sensor trigger is triggered or not

    final double ODSLightThreshold = 0.5; // When ODS is above this value we know that it is capturing the white line

    final double PowerAdjustCeiling = 0.2; // maximum we want to lower / higher the value of the motors

    /////////////////////////////////////////TELEOP VARIABLES/////////////////////////////////////

    boolean MotorSlowing = false; //Variable Setup// Press button to slow down motors by .25
    boolean Reversing = false; // reverses all motors to drive in backwards orientation
    double JoystickScalingFactor = 1;
    double DriveSlowingFactor = 0.25; // if motor slowing is activated

    final double FlapperPower = 1; // sets the spee of the flapper
    double LauncherPower = .22;//Was .22

    boolean flapperTriggered = false;



    public void AutonomyMotorAndSensorSetup(HardwareMap HWMAP) throws InterruptedException {
        hardwareMap = HWMAP;
        Left = hardwareMap.dcMotor.get("Left");  // Motor setup
        Right = hardwareMap.dcMotor.get("Right");
        Left.setDirection(DcMotor.Direction.REVERSE);
        Right.setDirection(DcMotor.Direction.FORWARD);

        Pusher = hardwareMap.servo.get("Pusher");

        Flapper = hardwareMap.dcMotor.get("Flapper");
        Launcher1 = hardwareMap.dcMotor.get("Launcher1");
        Launcher2 = hardwareMap.dcMotor.get("Launcher2");
        Lift = hardwareMap.dcMotor.get("Lift");
        Launcher1.setDirection(DcMotor.Direction.REVERSE);
        Launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Launcher2.setDirection(DcMotor.Direction.FORWARD);
        Launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift.setDirection(DcMotor.Direction.FORWARD);
        Flapper.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("Got Motors, Servos");
        telemetry.update();

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"range");

        InnerRightLight = hardwareMap.opticalDistanceSensor.get("InnerRightLight"); // Sensor setup
        InnerLeftLight = hardwareMap.opticalDistanceSensor.get("InnerLeftLight");

        ////////////////////////////GYRO///////////////////////////
        Gyro = hardwareMap.gyroSensor.get("Gyro");
        gyro = (ModernRoboticsI2cGyro) Gyro;

        gyro.calibrate();

        //////////////////////////////SetServo Posititon  ////////////////////////////
        Pusher.setPosition(NeutralPosition);

    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}//end CommonVariables class
