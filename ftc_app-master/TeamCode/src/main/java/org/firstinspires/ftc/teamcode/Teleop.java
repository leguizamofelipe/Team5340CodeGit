package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends CommonFunctions{
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomyMotorAndSensorSetup();

        waitForStart();

        while (opModeIsActive()) {
            ////////////////////////////////////////CONTROL OF VARIABLES - FROM GAMEPAD/////////////////////////////////
//            if (gamepad1.y){          //Slow Motor Toggle Control - Left Button
//                if (!MotorSlowing) {
//                    MotorSlowing = true;
//                } else {
//                    MotorSlowing = false;
//                }
//                sleep(500);
//            }
//
//            if(MotorSlowing){
//                SlowingFactor = 0.5;
//            }else{
//                SlowingFactor = 1;
//            }

            if (gamepad1.y){         //Reversing Toggle Control - Right Button
                if (Reversing == 1){
                    Reversing = -1;
                }else{
                    Reversing = 1;
                }
                sleep(500);
            }

            if(gamepad2.right_bumper) {                     //Flapper control - toggle using A
                if (Flapper.getPower() == 0) {
                    Flapper.setPower(FlapperPower);
                    FlapperTriggered = true;
                } else {
                    FlapperTriggered = false;
                    Flapper.setPower(0);
                }
                sleep(500);
            }

            if (gamepad2.left_bumper){
                if (Launcher1.getPower() == 0){
                    Launcher1.setPower(LauncherPower);
                    Launcher2.setPower(LauncherPower);
                } else {
                    Launcher1.setPower(0);
                    Launcher2.setPower(0);
                }
                sleep(500);
            }

            if(gamepad2.right_trigger != 0){
                Lift.setPower(gamepad2.right_trigger);
            } else if (gamepad2.left_trigger != 0) {
                Lift.setPower(-gamepad2.left_trigger);
                Flapper.setPower(-gamepad2.left_trigger);
            }else if (!FlapperTriggered){
                Flapper.setPower(0);
                Lift.setPower(0);
            }else{
                Lift.setPower(0);
            }

            if(gamepad2.x){
                Pusher.setPosition(PushLeft);
                sleep(500);
            } else if (gamepad2.b) {
                Pusher.setPosition(PushRight);
                sleep(500);
            } else if (gamepad2.a){
                Pusher.setPosition(NeutralPosition);
                sleep(500);
            }else{

            }

            ////////////////////////////////////////////////////////////

            double RightGamepadVal = gamepad1.right_stick_y * JoystickScalingFactor * Reversing; //Get Joystick values
            double LeftGamepadVal = gamepad1.left_stick_y * JoystickScalingFactor * Reversing;

                Left.setPower(LeftGamepadVal);
                Right.setPower(RightGamepadVal);
        }
    }

    public boolean WithinRange (double Target, double CurrentPos){
        if((Math.abs(Target - CurrentPos) < 0.05)){
            return true;
        }else{
            return false;
        }
    }
}
