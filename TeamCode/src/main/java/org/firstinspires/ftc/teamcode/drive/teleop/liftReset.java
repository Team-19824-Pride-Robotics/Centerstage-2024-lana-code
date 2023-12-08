package org.firstinspires.ftc.teamcode.drive.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;


@Config
@TeleOp (group = "teleop")
public class liftReset extends OpMode {
    ServoImplEx servo;
    ServoImplEx servo2;

    // AnalogInput sEncoder;
   AnalogInput sEncoder;
    AnalogInput sEncoder2;

    private static PIDController controller;
    public static double p = 0.005, i = 0, d =0;
    public static double f = 0;
    public static double target = 110;
    public static double liftpos1;
    public static double liftpos2;

    //lift
    static DcMotorEx lift1;
    static DcMotorEx lift2;

    @Override
    public void init() {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        servo = (ServoImplEx) hardwareMap.get(Servo.class, "Arm");
        servo.setPwmRange(new PwmControl.PwmRange(505, 2495));
        sEncoder = hardwareMap.get(AnalogInput.class, "sEncoder");
        servo2 = (ServoImplEx) hardwareMap.get(Servo.class, "bucket");
        servo2.setPwmRange(new PwmControl.PwmRange(505, 2495));
        sEncoder2 = hardwareMap.get(AnalogInput.class, "sEncoder2");


        controller = new PIDController(p, i, d);

        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setDirection(DcMotorEx.Direction.REVERSE);

    }

    @Override
    public void loop() {

        double pos = sEncoder.getVoltage() / 3.3 * 360;
        double pos2 = sEncoder2.getVoltage() / 3.3 * 360;



            servo2.setPosition(.09);
        if (pos2 >= 30 && pos2 <=34){
            servo.setPosition(.12);
        }
        if (pos>= 40 && pos<=44) {
            target = 0;
        }

        controller.setPID(p, i, d);
        int liftPos1 = lift1.getCurrentPosition();
        int liftPos2 = lift2.getCurrentPosition();
        double pid = controller.calculate(liftPos1, target);
        double pid2 = controller.calculate(liftPos2, target);
        double ff = 0;

        double lPower1 = pid +ff;
        double lPower2 = pid2 +ff;

        lift1.setPower(lPower1);
        lift2.setPower(lPower2);


        telemetry.addData("Run time",getRuntime());
        telemetry.addData("pos1", pos);
        telemetry.addData("pos2", pos2);
        telemetry.addData("test", 2);
        telemetry.update();
    }


}