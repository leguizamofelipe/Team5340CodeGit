package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import for_camera_opmodes.RunCamera;

/**
 * Created by RobotAdmin on 3/14/2017.
 */

public class CommonFunctions extends RunCamera {
    //TODO Variables

    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////VARIABLES /////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////
    
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

    final int DriveBackDistance = 12; //was 5 // after hitting first beacon how far we should back up

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
    public DcMotor Left = null;
    public DcMotor Right = null;

    public DcMotor Flapper = null;
    public DcMotor Launcher1 = null;
    public DcMotor Launcher2 = null;
    public DcMotor Lift = null;

    OpticalDistanceSensor InnerRightLight;
    OpticalDistanceSensor InnerLeftLight;
    OpticalDistanceSensor CenterOfRotation;
    OpticalDistanceSensor AlignmentGuide;

    GyroSensor Gyro;
    ModernRoboticsI2cGyro gyro;

//    I2cDevice frontDistanceSensor;
//    I2cDevice rearDistanceSensor;

    I2cDeviceSynch frontDistanceSensorReader;
   // I2cAddr frontDistanceSensorAdress = new I2cAddr(0x26);
   // I2cAddr rearDistanceSensorAdress = new I2cAddr(0x28);
    I2cDeviceSynch rearDistanceSensorReader;

    Servo Pusher;


    ///////////////////////////////////SENSOR VALUES/////////////////////////////////////////
    final double ODSLightThreshold = 0.5; // When ODS is above this value we know that it is capturing the white line

    final double PowerAdjustCeiling = 0.2; // maximum we want to lower / higher the value of the motors


    private boolean InnerLeftSensorTriggered = false;
    private boolean InnerRightSensorTriggered = false;

    byte[] frontDistanceSensorCache;
    byte[] rearDistanceSensorCache;

    int frontDistanceSensorRegStart = 0x04;
    int frontDistanceSensorReadLength = 2;

    int rearDistanceSensorRegStart = 0x04;
    int rearDistanceSensorReadLength = 2;

    /////////////////////////////////////////TELEOP VARIABLES/////////////////////////////////////

    boolean MotorSlowing = false; //Variable Setup// Press button to slow down motors by .25
    boolean Reversing = false; // reverses all motors to drive in backwards orientation
    double JoystickScalingFactor = 1;
    double SlowingFactor = 0.25;
    double DriveSlowingFactor = 0.25; // if motor slowing is activated

    final double FlapperPower = 1; // sets the spee of the flapper
    double LauncherPower = .22;//Was .22

    boolean FlapperTriggered = false;

    //TODO Motor And Sensor Setup

    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////MOTOR AND SENSOR SETUP/////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////

    public void AutonomyMotorAndSensorSetup() throws InterruptedException {

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


        /*
        frontDistanceSensor = hardwareMap.i2cDevice.get("frontDistanceSensor");
        rearDistanceSensor = hardwareMap.i2cDevice.get("rearDistanceSensor");

        frontDistanceSensorReader = new I2cDeviceSynchImpl(frontDistanceSensor, I2cAddr.create8bit(0x26), false);
        rearDistanceSensorReader = new I2cDeviceSynchImpl(rearDistanceSensor, I2cAddr.create8bit(0x28), false);

        frontDistanceSensorReader.engage();
        rearDistanceSensorReader.engage();

        */
       // rearDistanceSensor.



        //InnerRightLight = hardwareMap.opticalDistanceSensor.get("InnerRightLight"); // Sensor setup
        //InnerLeftLight = hardwareMap.opticalDistanceSensor.get("InnerLeftLight");

        InnerRightLight = hardwareMap.opticalDistanceSensor.get("InnerRightLight"); // Sensor setup
        InnerLeftLight = hardwareMap.opticalDistanceSensor.get("InnerLeftLight");
        CenterOfRotation = hardwareMap.opticalDistanceSensor.get("CenterOfRotation");
        AlignmentGuide = hardwareMap.opticalDistanceSensor.get("AlignmentGuide");

        ////////////////////////////GYRO///////////////////////////
        Gyro = hardwareMap.gyroSensor.get("Gyro");
        gyro = (ModernRoboticsI2cGyro) Gyro;


        gyro.calibrate();

        //////////////////////////////SetServo Posititon  ////////////////////////////
        Pusher.setPosition(NeutralPosition);

    }

    public void TeleopMotorAndSensorSetup() throws InterruptedException{
        Left = hardwareMap.dcMotor.get("Left");  //Motor setup
        Right = hardwareMap.dcMotor.get("Right");

        Flapper = hardwareMap.dcMotor.get("Flapper");
        Launcher1 = hardwareMap.dcMotor.get("Launcher1");
        Launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Launcher2 = hardwareMap.dcMotor.get("Launcher2");
        Launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift = hardwareMap.dcMotor.get("Lift");

        Pusher = hardwareMap.servo.get("Pusher");

        Left.setDirection(DcMotor.Direction.REVERSE);
        Right.setDirection(DcMotor.Direction.FORWARD);

        Launcher1.setDirection(DcMotor.Direction.REVERSE);
        Launcher2.setDirection(DcMotor.Direction.FORWARD);

        Lift.setDirection(DcMotor.Direction.FORWARD);
        Flapper.setDirection(DcMotorSimple.Direction.FORWARD);

        Pusher.setPosition(NeutralPosition);

    }


    //TODO Driving

    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////DRIVING FUNCTIONS//////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////

    //--------------------------------------------------------------------------
    //Convert distance to clicks
    //--------------------------------------------------------------------------
    private int DistanceToClicksConverter(double DistanceToTravel) {

        double DistancePerRotation = WheelDiameter * Math.PI;
        double RotationsNeeded = (-DistanceToTravel / DistancePerRotation);
        int ClicksNeeded = (int) (RotationsNeeded * ClicksPerRotation * GearRatio);

        return ClicksNeeded;
    }//end Distatnce to clicks Converter

    //--------------------------------------------------------------------------
    //Find how many degrees we are from the band of acceptable values
    //--------------------------------------------------------------------------
    private int DegreesFromTolerance(int Target, int Current){
        if(Math.abs(Target - Current) < DegreesOffTolerance){
            return 0;
        } else{
            return Math.abs(Target - Current) - DegreesOffTolerance;
        }
    }///end DegreesFromTolerance formula

    //--------------------------------------------------------------------------
    //Drive forward a set distance using encoders with the gyro keeping on angle
    //--------------------------------------------------------------------------
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
//            DriveStraightWithGyro(degreeToStayAt, Speed);
        }

        Left.setPower(0);
        Right.setPower(0);

        //reset mode as a safety
        Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //--------------------------------------------------------------------------
    //Drive backward a set distance using encoders with the gyro keeping on angle
    //--------------------------------------------------------------------------
    public void DriveBackwardWithEncoder(double DistanceToTravel, double Speed) throws InterruptedException {
        DriveForwardWithEncoder(-DistanceToTravel, -Speed);
    }//end DriveBackwardWithEncoder function

    //--------------------------------------------------------------------------
    //Drive forward until the distance sensor hits a set distance
    //--------------------------------------------------------------------------
//    public void DriveForwardWithDistanceSensor(double distanceToStop){
//        double currentDistance = frontDistanceSensor.getDistance(DistanceUnit.CM);;
//        int targetAngle = gyro.getIntegratedZValue();
//
//        while(currentDistance > distanceToStop){
//            if((currentDistance - distanceToStop) > slowDownDistance){
//                DriveStraightWithGyro(targetAngle, FAST_POWER);
//            }else{
//                DriveStraightWithGyro(targetAngle, SLOW_POWER);
//            }
//            currentDistance = frontDistanceSensor.getDistance(DistanceUnit.CM);
//        }//end while
//
//        Left.setPower(0);
//        Right.setPower(0);
//
//        Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }//end DriveForwardWithDistanceSensor

    //--------------------------------------------------------------------------
    //Turn until the gyro hits a certain absolute value
    ///If Turning Direction is 1 it means tur to the right and if turning direction is -1 it means turn left
    //--------------------------------------------------------------------------
    private void turn(int desiredAngle, int TurningDirection) throws InterruptedException { //Turning Direction = 1 when turning Right and turning Direction = -1 when turning Left
        gyro.resetZAxisIntegrator(); // resets to zero so the robots heading is relative to where we start are turn

        int currentDegree = Math.abs(gyro.getIntegratedZValue());
        int initialDegree = Math.abs(gyro.getIntegratedZValue());

        while (currentDegree < Math.abs(initialDegree + desiredAngle)) {
            telemetry.addData("ZVALUE:", Math.abs(gyro.getIntegratedZValue()));
            double power = 0.0;

            if(currentDegree < (Math.abs(initialDegree + desiredAngle) - 35)){ //Was at 25
                power = 0.15; //Was at .7
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

    //--------------------------------------------------------------------------
    //Use the master turn function to turn right
    //--------------------------------------------------------------------------
    public void turnRight(int DesiredDegree) throws InterruptedException{
        turn(DesiredDegree, 1);
    }

    //--------------------------------------------------------------------------
    //Use the master turn function to turn left
    //--------------------------------------------------------------------------
    public void turnLeft(int DesiredDegree) throws InterruptedException{
        turn(DesiredDegree,-1);
    }

    //--------------------------------------------------------------------------
    //Use the gyro to keep straight
    //--------------------------------------------------------------------------
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

    //--------------------------------------------------------------------------
    //Use the range to keep straight
    //--------------------------------------------------------------------------
    public void KeepStraightUsingRangeSensors(){
        frontDistanceSensorCache = frontDistanceSensorReader.read(0x04, 2);

        sleep(100);
        rearDistanceSensorCache = rearDistanceSensorReader.read(0x04, 2);

//        telemetry.addData("f name ", frontDistanceSensor.getDeviceName());
//        telemetry.addData("r nane",  rearDistanceSensor.getDeviceName());
            if (frontDistanceSensorCache[0] > rearDistanceSensorCache[0]) {
                telemetry.addData("DATA OF FRONT SENSOR: ", frontDistanceSensorCache[0] & 0xFF);
                telemetry.addData("DATA OF REAR SENSOR: ", rearDistanceSensorCache[0] & 0xFF);
                telemetry.addData("DATA OF ODS FRONT SENSOR: ", frontDistanceSensorCache[1] & 0xFF);
                telemetry.addData("DATA OF ODS REAR SENSOR: ", rearDistanceSensorCache[1] & 0xFF);
                telemetry.addLine("TURN LEFT");
                telemetry.update();
            } else if (frontDistanceSensorCache[0] < rearDistanceSensorCache[0]) {
                telemetry.addData("DATA OF FRONT SENSOR: ", frontDistanceSensorCache[0] & 0xFF);
                telemetry.addData("DATA OF REAR SENSOR: ", rearDistanceSensorCache[0] & 0xFF);
                telemetry.addData("DATA OF ODS FRONT SENSOR: ", frontDistanceSensorCache[1] & 0xFF);
                telemetry.addData("DATA OF ODS REAR SENSOR: ", rearDistanceSensorCache[1] & 0xFF);
                telemetry.addLine("TURN RIGHT");
                telemetry.update();
            } else {
                telemetry.addData("DATA OF FRONT SENSOR: ", frontDistanceSensorCache[0] & 0xFF);
                telemetry.addData("DATA OF REAR SENSOR: ", rearDistanceSensorCache[0] & 0xFF);
                telemetry.addData("DATA OF ODS FRONT SENSOR: ", frontDistanceSensorCache[1] & 0xFF);
                telemetry.addData("DATA OF ODS REAR SENSOR: ", rearDistanceSensorCache[1] & 0xFF);
                telemetry.addLine("YOU ARE EQUAL");
                telemetry.update();
            }
            frontDistanceSensorCache = frontDistanceSensorReader.read(frontDistanceSensorRegStart,frontDistanceSensorReadLength);

            sleep(100);
            rearDistanceSensorCache = rearDistanceSensorReader.read(rearDistanceSensorRegStart,rearDistanceSensorReadLength);
    }


    //TODO Interaction with Line and Beacon

    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////INTERACTIONS WITH LINE AND BEACON//////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////

    //--------------------------------------------------------------------------
    //Stop motors and wait a set amount of time
    //--------------------------------------------------------------------------
    public void StopAndWait(int TimeToWait) {
        Left.setPower(0);
        Right.setPower(0);
        sleep(TimeToWait); // Stop and think
    }//end StopAndWait

    //--------------------------------------------------------------------------
    //Booleans that return a value depending on the light threshold of the ODS Sensors
    //--------------------------------------------------------------------------
    boolean InnerRightDetectsLight() {
        if (InnerRightLight.getLightDetected() > ODSLightThreshold) {
            return true;
        } else {
            return false;
        }
    }//end InnerRightDetectsLight

    boolean InnerLeftDetectsLight() {
        if (InnerLeftLight.getLightDetected() > ODSLightThreshold) {
            return true;
        } else {
            return false;
        }
    }//end InnerLeftDetectsLight

    boolean CenterOfRotationDetectsLight() {
        if (CenterOfRotation.getLightDetected() > ODSLightThreshold) {
            return true;
        } else {
            return false;
        }
    }

    boolean AlignmentGuideDetectsLight() {
        if (AlignmentGuide.getLightDetected() > ODSLightThreshold) {
            return true;
        } else {
            return false;
        }
    }

    public void AlignWithLine(String AllianceColor, double Power) {
        if (AllianceColor.equals("RED")) {
            while(!CenterOfRotationDetectsLight()){
                Right.setPower(-Power);
                Left.setPower(-Power);
            }
            StopAndWait(200);

//            while (!InnerRightDetectsLight()) {// && !OuterRightDetectsLight()) {
//
//                telemetry.addLine("In !InnerRight");
//                telemetry.update();
//                if (InnerLeftDetectsLight()) {
//                    InnerLeftSensorTriggered = true;
//                }
//
//                if (!InnerLeftSensorTriggered) {
//                    Right.setPower(-FastSnapSpeed);
//                    Left.setPower(FastSnapSpeed);
//                    telemetry.addLine("Fast snap turn");
//                    telemetry.update();
//                } else if (InnerLeftSensorTriggered){
//                    Right.setPower(-SlowSnapSpeed);
//                    Left.setPower(SlowSnapSpeed);
//                    telemetry.addLine("Slow Snap turn");
//                    telemetry.update();
//                }
//            }

            while(!AlignmentGuideDetectsLight()){ //turn left
                Right.setPower(-Power);
                Left.setPower(Power);
            }
            StopAndWait(200);

        }else if (AllianceColor.equals("BLUE")){
            while(!CenterOfRotationDetectsLight()){
                Right.setPower(Power);
                Left.setPower(Power);
            }
            StopAndWait(200);

            while(!AlignmentGuideDetectsLight()){
                Right.setPower(-Power);
                Left.setPower(Power);
            }
            StopAndWait(200);
        }
    }

    //--------------------------------------------------------------------------
    //Intercept the line using the ODS sensor
    //--------------------------------------------------------------------------
    public void InterceptLine(String Determinant) throws InterruptedException { //Intercept line for the first time
        if (Determinant.equals("RED")) {
            while (!InnerLeftDetectsLight()){// && !OuterLeftDetectsLight()) { // Initial drive -- needed to catch the white tape
                Left.setPower(PowerForInterceptingDrive); // Was -.18
                Right.setPower(PowerForInterceptingDrive);// Was -.18
                
            }
        } else if (Determinant.equals("BLUE")) {
            while (!InnerRightDetectsLight()) {// && !OuterRightDetectsLight()) { // Initial drive -- needed to catch the white tape
                Left.setPower(PowerForInterceptingDrive);
                Right.setPower(PowerForInterceptingDrive);
            }
        }

        StopAndWait(250);
    }

    //--------------------------------------------------------------------------
    //Capture the line in between the two ODS sensors on the front
    //--------------------------------------------------------------------------
    public void SnapBackToLine(String Determinant) throws InterruptedException {
        if (Determinant.equals("RED")) {

            if (firstLineDone){
                DriveForwardWithEncoder(2, 0.3);
            }

            while (!InnerRightDetectsLight()) {// && !OuterRightDetectsLight()) {

                telemetry.addLine("In !InnerRight");
                telemetry.update();
                if (InnerLeftDetectsLight()) {
                    InnerLeftSensorTriggered = true;
                }

                if (!InnerLeftSensorTriggered) {
                    Right.setPower(-FastSnapSpeed);
                    Left.setPower(FastSnapSpeed);
                    telemetry.addLine("Fast snap turn");
                    telemetry.update();
                } else if (InnerLeftSensorTriggered){
                    Right.setPower(-SlowSnapSpeed);
                    Left.setPower(SlowSnapSpeed);
                    telemetry.addLine("Slow Snap turn");
                    telemetry.update();
                }
            }
        } else if (Determinant.equals("BLUE")) {

            if (firstLineDone){
                DriveForwardWithEncoder(1, 0.3);
            }

            while (!InnerLeftDetectsLight()) {// && !OuterLeftDetectsLight()) {

                if (InnerRightDetectsLight()){
                    InnerRightSensorTriggered = true;
                }
                if (!InnerRightSensorTriggered) {
                    Right.setPower(FastSnapSpeed);
                    Left.setPower(-FastSnapSpeed);
                } else {
                    Right.setPower(SlowSnapSpeed);
                    Left.setPower(-SlowSnapSpeed);
                }
            }
        }

        StopAndWait(250);
        // ResetTriggers();

    }
    //--------------------------------------------------------------------------
    //Use the ODS sensors to track the line and approach the beacon
    //--------------------------------------------------------------------------
    public void StraightenUpWithLine(double Speed) throws InterruptedException {
        DriveBackwardWithEncoder(3, 0.3);

        boolean LinedUp = false;

        while (!InnerLeftDetectsLight() && !InnerLeftDetectsLight()) { //Both detect dark, so drive forward
            Right.setPower(-Speed);
            Left.setPower(-Speed);
            telemetry.addLine("Both dark");
        }

        Right.setPower(0);
        Left.setPower(0);

        sleep(700);

        while(!LinedUp) {
            if (InnerLeftDetectsLight() && !InnerRightDetectsLight()) { //Right is dark, left is light
                Right.setPower(0);
                Left.setPower(-Speed);
                telemetry.addLine("Right Light, Dark");
            } else if (!InnerLeftDetectsLight() && InnerRightDetectsLight()) { //Right is light, left is dark
                Right.setPower(-Speed);
                Left.setPower(0);
                telemetry.addLine("Right Dark, Light");
            } else if (InnerLeftDetectsLight() && InnerRightDetectsLight()) {
                Right.setPower(0);
                Left.setPower(0);
                LinedUp = true;
                telemetry.addLine("Both Light, Both Light");
            }
        }

        Right.setPower(0);
        Left.setPower(0);
    }


    //--------------------------------------------------------------------------
    //Use the ODS sensors to track the line and approach the beacon
    //--------------------------------------------------------------------------
    //We assume the robot is relatively squared up on line, and that the line is captured between the ODS sensors
    public void TrackLineInwards() {
//        int currentDistance = frontDistanceSensor.getCallbackCount();

        int currentDistance = 5;
        while (currentDistance > StopDistanceFromWall && opModeIsActive()) {
            if (!InnerRightDetectsLight() && !InnerLeftDetectsLight() && opModeIsActive()) { // Both detect dark values, drive forward
                telemetry.addLine("Both are dark");
                Right.setPower(ForwardDrivingSpeed);
                Left.setPower(ForwardDrivingSpeed);
            } else if (InnerRightDetectsLight() && !InnerLeftDetectsLight() && opModeIsActive()) { //Right is light, left is dark, turn left
                telemetry.addLine("Turn Right");
                Right.setPower(TurningTrackSpeed);
                Left.setPower(-TurningTrackSpeed);
            } else if (!InnerRightDetectsLight() && InnerLeftDetectsLight() && opModeIsActive()) { //Right is light and left is dark
                telemetry.addLine("Turn Left");
                Right.setPower(-TurningTrackSpeed);
                Left.setPower(TurningTrackSpeed);
            } else if (InnerLeftDetectsLight() && InnerRightDetectsLight() && opModeIsActive()) { //Both are light
                telemetry.addLine("Both Light");
                Right.setPower(ForwardDrivingSpeed);
                Left.setPower(ForwardDrivingSpeed);
            } else {
                telemetry.addLine("Nothing!");
            }

            sleep(RunTimeMsec);

            Right.setPower(0);
            Left.setPower(0);
//            currentDistance = frontDistanceSensor.getCallbackCount();
        }
    }//end TrackLine Inwards Function

    //--------------------------------------------------------------------------
    //Use the camera to detect the color, then push the button
    //--------------------------------------------------------------------------
    public void PushButton(String AllianceColor) throws InterruptedException {
        if(AllianceColor.equals(colorString)){
            Pusher.setPosition(PushRight);
        }else{
            Pusher.setPosition(PushLeft);
        }
        sleep(500);
        DriveForwardWithEncoder(ButtonPushDriveDistance, PushingDriveSpeed);

        Pusher.setPosition(NeutralPosition);
        sleep(200);
    }
    
    //TODO Shooting Mechanism

    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////OPERATE SHOOTING MECHANISM/////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////

    //--------------------------------------------------------------------------
    //Run the components of the shooting mechanism, self explanatory
    //--------------------------------------------------------------------------
    public void turnOnFlapper() throws InterruptedException {
        Flapper.setPower(FlapperPower);
    }


    public void TurnOffFlapper()throws InterruptedException {
        Flapper.setPower(0);
    }

    public void TurnOnLaunchers() throws InterruptedException{
        Launcher1.setPower(LauncherPowerForAuto);
        Launcher2.setPower(LauncherPowerForAuto);
        telemetry.addLine("Running PID if same for two batteries!");
        telemetry.addData("Launcher1 Encoder Position", Launcher1.getCurrentPosition());
        telemetry.addData("Launcher2 Encoder Position", Launcher2.getCurrentPosition());
        telemetry.update();
    }

    public void TurnOffLaunchers() throws InterruptedException {
        Launcher1.setPower(0);
        Launcher2.setPower(0);
    }

    public void TurnOffLift() throws InterruptedException {
        Lift.setPower(0);
    }


    public void TurnOnLift() throws InterruptedException {
        Lift.setPower(LiftPower);
    }

    public void shootBalls() throws InterruptedException{
        TurnOnLaunchers();
        sleep(500);
        turnOnFlapper();
        TurnOnLift();

    }

    public void stopShootingBalls() throws InterruptedException {
        TurnOffFlapper();
        TurnOffLaunchers();
        TurnOffLift();
    }

    //--------------------------------------------------------------------------
    //Stop all motors
    //--------------------------------------------------------------------------
    public void KillMotors() throws InterruptedException{
        Right.setPower(0);
        Left.setPower(0);
        TurnOffLaunchers();
        TurnOffFlapper();
    }//end kill motors
}
