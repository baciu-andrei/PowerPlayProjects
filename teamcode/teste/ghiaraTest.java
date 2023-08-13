package org.firstinspires.ftc.teamcode.teste;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class ghiaraTest extends LinearOpMode {
    private static final double closedClaw = 0.35, openedClaw = 0.12;

    public void runOpMode() throws InterruptedException {

        boolean prev=true;
        boolean prev1=true;

        Servo claw=hardwareMap.get(Servo.class,"claw");
        claw.setPosition(closedClaw);
        waitForStart();
       while(opModeIsActive()){
           if(gamepad1.left_bumper)
               claw.setPosition(claw.getPosition()+0.01);
           if(gamepad1.right_bumper)
               claw.setPosition(claw.getPosition()-0.01);

           telemetry.addData("pozitiones des clawes",claw.getPosition());
           telemetry.addData("plm",prev);
           telemetry.update();
       }
    }
}
