package org.firstinspires.ftc.teamcode.teste;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp

public class liftTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, "motor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.4);

        waitForStart();
        while(opModeIsActive())
        {
            if(gamepad1.a)
                liftMotor.setPower(0.8);
            if(gamepad1.b)
                liftMotor.setPower(0.4);
            if(gamepad1.y)
                liftMotor.setPower(1);
            if(gamepad1.left_bumper)
            {

                liftMotor.setTargetPosition(liftMotor.getTargetPosition()+10);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor.setPower(0.5);
            }
            if(gamepad1.right_bumper)
            {
                liftMotor.setTargetPosition(liftMotor.getTargetPosition()-10);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor.setPower(0.5);
            }
            if(gamepad1.x)
            {
                liftMotor.setTargetPosition(300);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor.setPower(0.5);
            }
            telemetry.addData("curent",liftMotor.getCurrentPosition());
            telemetry.addData("target",liftMotor.getTargetPosition());
            telemetry.update();

        }
    }
}
