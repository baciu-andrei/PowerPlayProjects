package org.firstinspires.ftc.teamcode.risky;

import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@SuppressWarnings("all")
@TeleOp(name = "risky teleop")
public class Risky extends LinearOpMode {

    private static final int lifterLow = 200, lifterMid = 600, lifterHigh = 1600;
    private static final double closedClaw = 0.35, openedClaw = 0.12;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx stSus = hardwareMap.get(DcMotorEx.class, "stSus");
        DcMotorEx stJos = hardwareMap.get(DcMotorEx.class, "stJos");
        DcMotorEx drSus = hardwareMap.get(DcMotorEx.class, "drSus");
        DcMotorEx drJos = hardwareMap.get(DcMotorEx.class, "drJos");
        stSus.setDirection(DcMotorSimple.Direction.REVERSE);
        stJos.setDirection(DcMotorSimple.Direction.FORWARD);
        drJos.setDirection(DcMotorSimple.Direction.FORWARD);
        drSus.setDirection(DcMotorSimple.Direction.REVERSE);
        stSus.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stJos.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drSus.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drJos.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stSus.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stJos.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drSus.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drJos.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stSus.setPower(0);
        stJos.setPower(0);
        drSus.setPower(0);
        drJos.setPower(0);

        DcMotorEx lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setPower(0);

        Servo claw = hardwareMap.get(Servo.class, "servoGhiara");
        claw.setDirection(Servo.Direction.FORWARD);
        claw.setPosition(openedClaw);

        IMU imu = hardwareMap.get(IMU.class, "imuControl");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        //initializari in pula mea
        boolean wasYPressedPreviously = false;
        boolean slow = false;
        boolean wasLbPressedPreviously = false;
        boolean wasDpadHighUsedPreviously = false;
        boolean wasDpadMidUsedPreviously = false;
        boolean wasDpadLowUsedPreviously = false;
        boolean wasYPressedPreviously2 = false;
        boolean isClawOpen = true;

        waitForStart();

        while(opModeIsActive())
        {
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotx = gamepad1.left_stick_x * Math.cos(-botHeading) - gamepad1.left_stick_y * Math.sin(-botHeading);
            double roty = gamepad1.left_stick_x * Math.sin(-botHeading) + gamepad1.left_stick_y * Math.cos(-botHeading);
            double impart = Math.max(1, Math.max(abs(-roty + rotx - gamepad1.left_trigger + gamepad1.right_trigger), Math.max(abs(-roty - rotx + gamepad1.left_trigger - gamepad1.right_trigger), Math.max(abs(-roty + rotx + gamepad1.left_trigger - gamepad1.right_trigger), abs(-roty - rotx - gamepad1.left_trigger + gamepad1.right_trigger)))));

            if (slow == true) {
                stSus.setPower(((-roty + rotx + gamepad1.left_trigger - gamepad1.right_trigger) / impart) / 3);
                drSus.setPower(((-roty - rotx - gamepad1.left_trigger + gamepad1.right_trigger) / impart) / 3);
                stJos.setPower(((-roty - rotx + gamepad1.left_trigger - gamepad1.right_trigger) / impart) / 3);
                drJos.setPower(((-roty + rotx - gamepad1.left_trigger + gamepad1.right_trigger) / impart) / 3);
            } else {
                stSus.setPower((-roty + rotx + gamepad1.left_trigger - gamepad1.right_trigger) / impart);
                drSus.setPower((-roty - rotx - gamepad1.left_trigger + gamepad1.right_trigger) / impart);
                stJos.setPower((-roty - rotx + gamepad1.left_trigger - gamepad1.right_trigger) / impart);
                drJos.setPower((-roty + rotx - gamepad1.left_trigger + gamepad1.right_trigger) / impart);
            }

            if (gamepad1.y && slow == false && !wasYPressedPreviously)
                slow = true;
            else if (gamepad1.y && slow == true && !wasYPressedPreviously)
                slow = false;
            wasYPressedPreviously = gamepad1.y;
            //movement code

            if (gamepad2.left_bumper && !wasLbPressedPreviously) {
                if (isClawOpen)
                    claw.setPosition(closedClaw);
                else
                    claw.setPosition(openedClaw);
                isClawOpen = ! isClawOpen;
            }

            wasLbPressedPreviously = gamepad2.left_bumper;
            //claw code
            if(gamepad2.dpad_up)
            {
                lift.setTargetPosition(lifterHigh);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(lift.getCurrentPosition()>lifterHigh)
                    lift.setPower(-.4);
                else lift.setPower(.4);
            }
            else if(gamepad2.dpad_left)
            {
                lift.setTargetPosition(lifterMid);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(lift.getCurrentPosition()>lifterMid)
                    lift.setPower(-.4);
                else lift.setPower(.4);
            }
            else if(gamepad2.dpad_down)
            {
                lift.setTargetPosition(lifterLow);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(lift.getCurrentPosition()>lifterLow)
                    lift.setPower(-.4);
                else lift.setPower(.4);
            }

            else if(gamepad2.y && !wasYPressedPreviously)
            {
                lift.setTargetPosition(30);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(.2);
            }

            wasYPressedPreviously2 = gamepad2.y;
            wasDpadHighUsedPreviously = gamepad2.dpad_up;
            wasDpadMidUsedPreviously = gamepad2.dpad_left;
            wasDpadLowUsedPreviously = gamepad2.dpad_down;
            //lift code

            if(gamepad2.left_trigger > 0.1)
            {
                int ticks = lift.getCurrentPosition() - 20;
                lift.setTargetPosition((ticks < 30)?(30):(ticks));
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(-.2);
            }
            else if(gamepad2.right_trigger > 0.1)
            {
                lift.setTargetPosition(lift.getCurrentPosition() + 20);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
            }
            //manual lift
        }
    }
}