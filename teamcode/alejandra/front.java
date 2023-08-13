package org.firstinspires.ftc.teamcode.alejandra;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class front {
    private Servo s1;
    private Servo s2;
    public front(HardwareMap hm){
        s1 = hm.get(Servo.class, "servo1");
        s2 = hm.get(Servo.class, "servo2");
        s1.setDirection(Servo.Direction.REVERSE);
        s2.setDirection(Servo.Direction.REVERSE);
    }
    public void setPozi(double poz)
    {
        s1.setPosition(poz);
        s2.setPosition(poz);
    }
    public double getPozi()
    {
        return s1.getPosition();
    }
}
