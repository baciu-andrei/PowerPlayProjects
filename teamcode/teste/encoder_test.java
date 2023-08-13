package org.firstinspires.ftc.teamcode.teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Steering calib")
public class encoder_test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class,"motor");

        while(opModeIsActive())
        {

            telemetry.addData("poz",motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
