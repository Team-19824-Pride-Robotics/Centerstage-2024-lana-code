package org.firstinspires.ftc.teamcode.drive.auto;



import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class arm {

    double aPos =.01;

    public static double bPosx = .2;
    ServoImplEx Arm;
    ServoImplEx bucket;


    public arm(HardwareMap hardwareMap) {
        Arm= (ServoImplEx) hardwareMap.get(Servo.class, "Arm");
        bucket = (ServoImplEx) hardwareMap.get(Servo.class, "bucket");
        Arm.setPwmRange(new PwmControl.PwmRange(505, 2495));
        bucket.setPwmRange(new PwmControl.PwmRange(505, 2495));
        }



    public void update() {

        Arm.setPosition(aPos);
        Arm.setPosition(bPosx);

    }
    public void bScore(){
        bPosx = .99;
    }
    public void bMiddle(){
        bPosx = .55;
    }
    public void bPickup(){
        bPosx = .2;
    }

    public void aPickup(){
        aPos = .01;
    }
    public void aScore(){
        aPos = .99;
    }
}