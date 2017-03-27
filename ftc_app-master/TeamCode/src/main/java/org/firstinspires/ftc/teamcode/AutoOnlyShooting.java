package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


/**
 * Created by RobotAdmin on 1/18/2017.
 */

@Autonomous(name="Only Shooting", group="Autonomous")
@Disabled
public class AutoOnlyShooting extends CommonFunctions{

    @Override
    public void runOpMode() throws InterruptedException{
        AutonomyMotorAndSensorSetup();

        waitForStart();

        sleep(10000);

        DriveBackwardWithEncoder(32, 0.3); //get in pos to shoot

        shootBalls(); //shoot
        sleep(6000);
        stopShootingBalls();

        DriveBackwardWithEncoder(25, 0.3); //get out of the way of partner
       // DriveForwardWithEncoder(13, 0.3);
    }
}
