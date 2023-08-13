package org.firstinspires.ftc.teamcode.teste;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class doubleServo {
    private Servo l;
    private Servo r;
    public doubleServo(HardwareMap hm){
        l=hm.get(Servo.class, "leftServo");
        r=hm.get(Servo.class, "rightServo");
    }
    public void setpoz(double val){
        l.setPosition(val);
        r.setPosition(val);
    }
    public double getpoz(){
        return l.getPosition();
    }
}
