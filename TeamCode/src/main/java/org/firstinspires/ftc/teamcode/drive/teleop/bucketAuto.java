package org.firstinspires.ftc.teamcode.drive.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;


@Config
@TeleOp (group = "teleop")
public class bucketAuto extends OpMode {
    ServoImplEx servo;
    ServoImplEx servo2;

    // AnalogInput sEncoder;
   AnalogInput sEncoder;
    AnalogInput sEncoder2;

    public static double bucket = .42;


    @Override
    public void init() {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        servo = (ServoImplEx) hardwareMap.get(Servo.class, "Arm");
        servo.setPwmRange(new PwmControl.PwmRange(505, 2495));
        sEncoder = hardwareMap.get(AnalogInput.class, "sEncoder");
        servo2 = (ServoImplEx) hardwareMap.get(Servo.class, "bucket");
        servo2.setPwmRange(new PwmControl.PwmRange(505, 2495));
        sEncoder2 = hardwareMap.get(AnalogInput.class, "sEncoder2");



    }

    @Override
    public void loop() {

        double pos = sEncoder.getVoltage() / 3.3 * 360;
        double pos2 = sEncoder2.getVoltage() / 3.3 * 360;



            servo2.setPosition(bucket);



        telemetry.addData("Run time",getRuntime());
        telemetry.addData("pos1", pos);
        telemetry.addData("pos2", pos2);
        telemetry.addData("test", 2);
        telemetry.update();
    }


}