package org.firstinspires.ftc.teamcode;

/**
 * Created by robotadmin on 3/10/2017.
 */

public class CommonShootingMechanismFunctions extends CommonMotorAndSensorSetup {


    public void turnOnFlapper() throws InterruptedException {
        commonVariables.Flapper.setPower(commonVariables.FlapperPower);
    }


    public void TurnOffFlapper()throws InterruptedException {
        commonVariables.Flapper.setPower(0);
    }

    public void TurnOnLaunchers() throws InterruptedException{
        commonVariables.Launcher1.setPower(commonVariables.LauncherPowerForAuto);
        commonVariables.Launcher2.setPower(commonVariables.LauncherPowerForAuto);
        telemetry.addLine("Running PID if same for two batteries!");
        telemetry.addData("Launcher1 Encoder Position", commonVariables.Launcher1.getCurrentPosition());
        telemetry.addData("Launcher2 Encoder Position", commonVariables.Launcher2.getCurrentPosition());
        telemetry.update();
    }

    public void TurnOffLaunchers() throws InterruptedException {
        commonVariables.Launcher1.setPower(0);
        commonVariables.Launcher2.setPower(0);
    }

    public void TurnOffLift() throws InterruptedException {
        commonVariables.Lift.setPower(0);
    }


    public void TurnOnLift() throws InterruptedException {
        commonVariables.Lift.setPower(commonVariables.LiftPower);
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

    /////////////Stops All Motors////////////////////////////////////

    public void KillMotors() throws InterruptedException{
        commonVariables.Right.setPower(0);
        commonVariables.Left.setPower(0);
        TurnOffLaunchers();
        TurnOffFlapper();
    }//end kill motors
}//End CommonShootingMechanismFunctions
