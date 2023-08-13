package org.firstinspires.ftc.teamcode.alejandra;

import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Disabled
public class alejandra extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        RevColorSensorV3 sensor = hardwareMap.get(RevColorSensorV3.class, "intakeSensor");
        RevColorSensorV3 sensor_pal = hardwareMap.get(RevColorSensorV3.class, "intakeSensorPal");
        DcMotorEx FrontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx FrontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx BottomLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx BottomRight = hardwareMap.get(DcMotorEx.class, "backRight");
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        BottomLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        BottomRight.setDirection(DcMotorSimple.Direction.REVERSE);
        IMU imu = hardwareMap.get(IMU.class, "imuControl");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        Servo claw = hardwareMap.get(Servo.class, "claw");
        Servo salam = hardwareMap.get(Servo.class, "salam");
        claw.setPosition(0.35);
        double salam_up = 0.04;
        double salam_down = 0.44;
        boolean prev = true;
        boolean prev2 = true;
        boolean prevs = true;
        boolean prevrb=false;
        boolean prevlb=false;
        int h1 = 180;
        int h2 = 600;
        int h3 = 925;
        int stackIndex [] = {0,0,20,60,120};
        double armIndex [] = {0,0.02,0.05,0.05,0.05};
        int i = 0;
        palnie pal = new palnie(hardwareMap);
        Brat arm = new Brat(hardwareMap);
        arm.setpoz(0);
        lift lif = new lift(hardwareMap);
        salam.setPosition(salam_up);
        lif.setpower(1);
        waitForStart();
        new Thread(() -> {

            boolean prev3 = true;
            boolean slow = false;
            while (opModeIsActive()) {
                {
                    if (gamepad1.options) {
                        imu.resetYaw();
                    }
                    double botHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    double rotx = gamepad1.left_stick_x * Math.cos(-botHeading) - gamepad1.left_stick_y * Math.sin(-botHeading);
                    double roty = gamepad1.left_stick_x * Math.sin(-botHeading) + gamepad1.left_stick_y * Math.cos(-botHeading);
                    double impart;
                    impart = Math.max(1, Math.max(abs(-roty + rotx - gamepad1.left_trigger + gamepad1.right_trigger), Math.max(abs(-roty - rotx + gamepad1.left_trigger - gamepad1.right_trigger), Math.max(abs(-roty + rotx + gamepad1.left_trigger - gamepad1.right_trigger), abs(-roty - rotx - gamepad1.left_trigger + gamepad1.right_trigger)))));
                    if (slow == true) {
                        FrontLeft.setPower(((-roty + rotx + gamepad1.left_trigger - gamepad1.right_trigger) / impart) / 3);
                        FrontRight.setPower(((-roty - rotx - gamepad1.left_trigger + gamepad1.right_trigger) / impart) / 3);
                        BottomLeft.setPower(((-roty - rotx + gamepad1.left_trigger - gamepad1.right_trigger) / impart) / 3);
                        BottomRight.setPower(((-roty + rotx - gamepad1.left_trigger + gamepad1.right_trigger) / impart) / 3);

                    } else {
                        FrontLeft.setPower((-roty + rotx + gamepad1.left_trigger - gamepad1.right_trigger) / impart * 0.9);
                        FrontRight.setPower((-roty - rotx - gamepad1.left_trigger + gamepad1.right_trigger) / impart * 0.9);
                        BottomLeft.setPower((-roty - rotx + gamepad1.left_trigger - gamepad1.right_trigger) / impart * 0.9);
                        BottomRight.setPower((-roty + rotx - gamepad1.left_trigger + gamepad1.right_trigger) / impart * 0.9);
                    }
                    if (gamepad1.y && slow == false && !prev3)
                        slow = true;
                    else if (gamepad1.y && slow == true && !prev3)
                        slow = false;
                    prev3 = gamepad1.y;
                }//movement code
            }
        }).start();
        boolean prev_x = false;

        while (opModeIsActive()) {
            double distance = sensor.getDistance(DistanceUnit.CM);
            double distance_pal = sensor_pal.getDistance(DistanceUnit.CM);
            {
                if (gamepad1.left_bumper && !prev) {
                    if (is_equal(claw.getPosition(), 0.22))
                        claw.setPosition(0.35);
                    else if (is_equal(claw.getPosition(), 0.35))
                        claw.setPosition(0.22);
                }
                prev = gamepad1.left_bumper;
            }//claw code
            //fostul brat code
            {
                if (gamepad2.dpad_up) {
                    lif.setTarget(h3);
                    pal.extinde();
                } else if (gamepad2.dpad_left) {
                    lif.setTarget(h2);
                    pal.extinde();
                } else if (gamepad2.dpad_down) {
                    lif.setTarget(h1);
                    pal.extinde();
                }
                if(gamepad2.dpad_right)
                {
                    lif.setTarget(0);
                    pal.extinde();//greseluta
                }
                else if(gamepad2.b)
                {
                    lif.setTarget(lif.getPozition()-50);
                    sleep(200);
                    claw.setPosition(0.35);
                    sleep(400);
                    claw.setPosition(0.26);
                    pal.retrage();
                    sleep(300);
                    arm.setpoz(0.475);
                    sleep(500);
                    if(i > 0)
                        i--;
                    lif.setTarget(stackIndex[i]);
                    while (lif.getPozition() > stackIndex[i] + 20 && opModeIsActive()) {
                        idle();
                    }

                    arm.setpoz(armIndex[i]);
                    sleep(200);
                    claw.setPosition(0.35);
                }//reset code
                if(distance_pal<1){
                    if(lif.getPozition()>h1-30&&lif.getPozition()<h1+30)
                    {
                        lif.setTarget(100);
                    }
                    if(lif.getPozition()>h2-30&&lif.getPozition()<h2+30)
                    {
                        lif.setTarget(450);
                    }
                    if(lif.getPozition()>h3-30&&lif.getPozition()<h3+30)
                    {
                        lif.setTarget(820);
                    }
                    sleep(500);
                    claw.setPosition(0.35);
                    sleep(400);
                    claw.setPosition(0.26);
                    pal.retrage();
                    sleep(300);
                    arm.setpoz(0.475);
                    sleep(500);
                    if(i > 0)
                        i--;
                    lif.setTarget(stackIndex[i]);
                    while (lif.getPozition() > stackIndex[i] + 20 && opModeIsActive()) {
                        idle();
                    }

                    arm.setpoz(armIndex[i]);
                    sleep(200);
                    claw.setPosition(0.35);
                }
                if(gamepad2.left_trigger>0.5)
                {

                    lif.setTarget(lif.getPozition()-50);
                }
                if(gamepad2.right_trigger>0.5)
                {
                    lif.setTarget(lif.getPozition()+50);
                }
                if(lif.getTarget()<0)
                    lif.setTarget(0);
            }//lift
            {
                if (gamepad1.x && prev_x != gamepad1.x) {
                    prevs = !prevs;
                }
                if (!prevs) {
                    salam.setPosition(salam_down);
                } else {
                    salam.setPosition(salam_up);
                }
                prev_x = gamepad1.x;
            }//salam
            {
                if (is_equal(claw.getPosition(), 0.22) && lif.getPozition() <= stackIndex[i] + 20 && is_equal(arm.getpoz(), armIndex[i])) {
                    sleep(300);
                    arm.setpoz(0.675);
                }
            }//dam pe spate automat
            {
                if (distance < 1.70) {
                    claw.setPosition(0.22);
                }
            }//inchidem ghieruta cand low distance
            {

                if(gamepad2.left_bumper&&!prevlb&&i<4)
                {
                    i++;
                    lif.setTarget(stackIndex[i]);
                    arm.setpoz(armIndex[i]);
                }
                if(gamepad2.right_bumper&&!prevrb&&i>=1)
                {
                    i--;
                    lif.setTarget(stackIndex[i]);
                    arm.setpoz(armIndex[i]);
                }
                prevlb=gamepad2.left_bumper;
                prevrb=gamepad2.right_bumper;

            }//conuri stackate
        }

    }
    boolean is_equal(double a, double b) {
        return abs(a - b) < 0.01;

    }//metoda de comp.
}