package org.firstinspires.ftc.teamcode.Bistrita;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BackSpeed{
    private DcMotorEx shaft;
    private DcMotorEx rb;
    private DcMotorEx lb;

    public BackSpeed(HardwareMap hm)
    {
        shaft = hm.get(DcMotorEx.class,"shaft");
        rb = hm.get(DcMotorEx.class,"rb");
        lb = hm.get(DcMotorEx.class,"lb");

        shaft.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);

        shaft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shaft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shaft.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }
    public void setPwr(double pwr)
    {
        shaft.setPower(pwr);
        lb.setPower(pwr);
        rb.setPower(pwr);
    }
}
