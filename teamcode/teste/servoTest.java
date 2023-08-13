package org.firstinspires.ftc.teamcode.teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.alejandra.front;

@TeleOp(name="servoTest")

public class servoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        front fro = new front(hardwareMap);
        fro.setPozi(0.5);
        waitForStart();
        while(opModeIsActive())
        {
            if(gamepad1.dpad_up) {
                fro.setPozi(fro.getPozi() + 0.001);
            }
            if(gamepad1.dpad_down){
                fro.setPozi(fro.getPozi()-0.001);
            }

            telemetry.addData("+poz",fro.getPozi());
            telemetry.update();

        }
    }
}
