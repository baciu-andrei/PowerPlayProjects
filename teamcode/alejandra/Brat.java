package org.firstinspires.ftc.teamcode.alejandra;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Brat {
    private Servo l;
    private Servo r;
    public Brat(HardwareMap hm) {
        l = hm.get(Servo.class, "leftArm");
        r = hm.get(Servo.class, "rightArm");
        r.setDirection(Servo.Direction.REVERSE);
    }

    public void setpoz(double val){
        l.setPosition(val);
        r.setPosition(val);
    }
    public double getpoz(){
        return l.getPosition();
    }
}