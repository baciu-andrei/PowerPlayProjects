package org.firstinspires.ftc.teamcode.risky;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class Doamne_ajuta extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx stSus = hardwareMap.get(DcMotorEx.class, "stSus");
        DcMotorEx stJos = hardwareMap.get(DcMotorEx.class, "stJos");
        DcMotorEx drSus = hardwareMap.get(DcMotorEx.class, "drSus");
        DcMotorEx drJos = hardwareMap.get(DcMotorEx.class, "drJos");
        stSus.setDirection(DcMotorSimple.Direction.REVERSE);
        stJos.setDirection(DcMotorSimple.Direction.FORWARD);
        drJos.setDirection(DcMotorSimple.Direction.REVERSE);
        drSus.setDirection(DcMotorSimple.Direction.REVERSE);
        stSus.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stJos.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drSus.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drJos.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stSus.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stJos.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drSus.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drJos.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while(opModeIsActive()) {
            stSus.setPower(0.3);
            stJos.setPower(0.3);
            drSus.setPower(0.3);
            drJos.setPower(0.3);
            sleep(1600);
            stSus.setPower(0);
            stJos.setPower(0);
            drSus.setPower(0);
            drJos.setPower(0);
        }
    }
}
