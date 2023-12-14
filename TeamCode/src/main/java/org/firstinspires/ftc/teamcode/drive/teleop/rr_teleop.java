package org.firstinspires.ftc.teamcode.drive.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;


@TeleOp(group="teleop")
public class rr_teleop extends LinearOpMode {


    public static double speed = 1;

    //pid
    private PIDController controller;
    public static double p = 0.005, i = 0, d =0;
    public static double f = 0;
    public static double target = 120;

    boolean liftControl = false;


    //intake
    DcMotorEx intake;

    public static double power = 1;
    public static double backPower =-1;
    //lift
    DcMotorEx lift1;
    DcMotorEx lift2;


    public static double liftM = 40;
    public static double liftMax = 2000;

    //winch
    DcMotorEx winch;

    public static int wBack = -200;
    public static int wPos = 0;
    public static int wSpeed = 200;
    public static int wUp = 2000;
    public static int wDown = 1500;
    public static double wbPower = .5;
    public static double wuPower = 1;
    public static double wdPower = 1;



    //arm

    double aPos =.08;

    public static double bPosx = .13;
    public static double bChange = .001;
    ServoImplEx Arm;
    ServoImplEx bucket;
    // AnalogInput sEncoder;
    AnalogInput sEncoder;
    AnalogInput sEncoder2;

    //drone
    Servo drone;
    public static double launch = .3;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        target = 110;
        aPos =.08;
        bPosx = .13;
        wPos = 0;


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//pid
        controller = new PIDController(p,i,d);


        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setDirection(DcMotorEx.Direction.REVERSE);


        Arm= (ServoImplEx) hardwareMap.get(Servo.class, "Arm");
        bucket = (ServoImplEx) hardwareMap.get(Servo.class, "bucket");
        Arm.setPwmRange(new PwmControl.PwmRange(505, 2495));
        bucket.setPwmRange(new PwmControl.PwmRange(505, 2495));
        sEncoder = hardwareMap.get(AnalogInput.class, "sEncoder");
        sEncoder2 = hardwareMap.get(AnalogInput.class, "sEncoder2");


        intake = hardwareMap.get(DcMotorEx.class, "intake");

        drone = hardwareMap.get(Servo.class, "drone");

        winch = hardwareMap.get(DcMotorEx.class, "winch");
        winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            Pose2d myPose = drive.getPoseEstimate();

            target =  Range.clip(target, 110, 2400);

            if (gamepad1.right_bumper) {
                speed = 1;
            }
            if (gamepad1.left_bumper) {
                speed = .5;
            } else {
                speed = .75;
            }

            double driving = (-gamepad1.left_stick_y) * speed;
            double strafing = (-gamepad1.left_stick_x) * 0;
            double turning = (-gamepad1.right_stick_x) * speed;

            if (gamepad1.right_bumper)
                driving = (-gamepad1.left_stick_y) * 1;

            if (gamepad1.left_trigger > 0.3) {
                strafing = (gamepad1.left_trigger) * 0.75;
            }
            if (gamepad1.right_trigger > 0.3) {
                strafing = (-gamepad1.right_trigger) * 0.75;
            }
            if (gamepad1.dpad_left) {
                strafing = -0.25;
            }
            if (gamepad1.dpad_right) {
                strafing = 0.25;
            }
            if (gamepad1.dpad_up) {
                driving = -0.25;
            }
            if (gamepad1.dpad_down) {
                driving = 0.25;
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            (driving),
                            (strafing),
                            (turning)
                    )
            );

            drive.update();
            //drone
            if (gamepad1.b){
                drone.setPosition(launch);
            }
            //winch
            if (gamepad1.a){
                wPos = winch.getCurrentPosition() + wSpeed;

            }
            if (gamepad1.x){
                wPos = winch.getCurrentPosition() + 50;

            }
            if (gamepad1.y){
                wPos = winch.getCurrentPosition() - wSpeed;
            }

            winch.setTargetPosition(wPos);
            winch.setPower(wdPower);
            winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //DRIVER 2//

//intake
            if (gamepad2.x) {
                intake.setPower(power);
            }
            if (gamepad2.b) {
                intake.setPower(backPower);
            }
            if (gamepad2.a){
                intake.setPower(0);
            }

            double pos = sEncoder.getVoltage() / 3.3 * 360;
            double pos2 = sEncoder2.getVoltage() / 3.3 * 360;

            aPos =  Range.clip(aPos, .01, .99);

            if (gamepad2.right_bumper){
                liftControl = true;
                target=300;
                bPosx=.45;
            }
            if (gamepad2.left_bumper){
                liftControl = false;
                bPosx=.13;
            }




            if (liftControl) {
                if (gamepad2.y){
                    aPos = .99;
                }
                if (gamepad2.left_stick_y<.2 || gamepad2.left_stick_y>.2){
                    target = -gamepad2.left_stick_y * liftM + target;
                }
                if (gamepad1.start) {
                    bPosx =.9;
                }
                if (gamepad1.share) {
                    bPosx =.55;
                }
                if (gamepad2.dpad_down) {
                    target =1100;
                }
                if (gamepad2.dpad_left) {
                    target =2000;
                }
                if (gamepad2.dpad_up) {
                    target =2400;
                }
                if(gamepad2.start && bucket.getPosition()<0.99){
                    bPosx = bucket.getPosition() + bChange;
                }
                if(gamepad2.share && bucket.getPosition()>0.01) {
                    bPosx = bucket.getPosition() - bChange;
                }
            }

            if (!liftControl) {
                if (pos2 >= 48 && pos2 <=52){
                    aPos = .08;
                }
                if (pos >= 32 && pos<= 36
                ) {
                    target = 120;
                }
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
            Arm.setPosition(aPos);
            bucket.setPosition(bPosx);

            telemetry.addData("target",target);
            telemetry.addData("Run time",getRuntime());
            telemetry.addData("1","test");
            telemetry.addData("pos1", lift1.getCurrentPosition());
            telemetry.addData("power1", lift1.getPower());
            telemetry.addData("pos2", lift2.getCurrentPosition());
            telemetry.addData("power2", lift2.getPower());
            telemetry.addData("arm", pos);
            telemetry.addData("bucket", pos2);
            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", myPose.getHeading());

            telemetry.update();
        }
    }
}
