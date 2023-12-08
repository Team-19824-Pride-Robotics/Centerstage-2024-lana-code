package org.firstinspires.ftc.teamcode.drive.auto;



import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class intake {


    public static double power = 0;

    //lift
    static DcMotorEx intake;

    public intake(HardwareMap hardwareMap) {
        //pid

            intake = hardwareMap.get(DcMotorEx.class, "lift1");
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setDirection(DcMotorEx.Direction.REVERSE);



        }

    public static void update() {

        intake.setPower(power);

    }
    public static void score() {
        power = -1;
    }
    public static void zero() {

        power = 0;
    }


}