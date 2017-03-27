package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestServo", group = "Concept")
public class TestServo extends OpMode { //fully up .5 dump .3 hold 1

    Servo Pusher;

    @Override
    public void init() {
        Pusher = hardwareMap.servo.get("Pusher");
    }

    @Override
    public void loop() {
        Pusher.setPosition(gamepad1.right_trigger);

        telemetry.addData("Pusher", Pusher.getPosition());
        telemetry.update();
    }
}
