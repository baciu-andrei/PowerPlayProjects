package org.firstinspires.ftc.teamcode.alejandra;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class palnie {
    private Servo palnie;
    public palnie(HardwareMap hm)
    {
        palnie=hm.get(Servo.class, "palnie");
        palnie.setPosition(0.25);
    }
    public void extinde()
    {
        palnie.setPosition(0.45);
    }
    public void retrage()
    {
        palnie.setPosition(0.25);
    }
}
//0.25
//0.45