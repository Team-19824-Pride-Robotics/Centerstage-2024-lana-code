package org.firstinspires.ftc.teamcode.drive.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@Config
@TeleOp(name = "teleop_v2", group = "teleop")
public class teleop_v2 extends LinearOpMode {

    //pid setup
    private PIDController controller;
    public static double p = 0.005, i = 0, d = 0;
    public static double f = 0;
    public static double target = 200;

    //intialize motors

    DcMotorEx FR;
    DcMotorEx FL;
    DcMotorEx BR;
    DcMotorEx BL;
    DcMotorEx intake;
    DcMotorEx lift1;
    DcMotorEx lift2;
    DcMotorEx winch;
    Servo arm;
    Servo bucket;
    Servo outtake_lid;
    Servo pincer_left;
    Servo pincer_right;
    Servo drone;

    //intake settings
    public static double power = 1;
    public static double backPower = -1;

    //lift settings
    public static double liftM = 40;

    //winch settings
    public static int wPos = 0;
    public static int wSpeed = 50;
    public static double wdPower = 1;

    //arm and bucket settings
    public static double aPos;
    public static double bPosx;
    public static double bChange = .001;

    //pincer setup
    public static double pince_time = 0.15;
    public static double right_open = 0.3;
    public static double right_closed = 0.12;
    public static double left_open = 0.69;
    public static double left_closed = 0.87;

    //drone setup
    public static double launch = .3;

    //bucket lid setup
    public static double out_shut = 0.58;
    public static double out_half = 0.65;
    public static double out_open = 0.8;

    //arm and bucket setup
    public static double bucket_score = 0.75;
    public static double bucket_intake = 0.47;
    public static double arm_intake = 0.99;
    public static double arm_score = 0.01;
    public static double lift_intake = 200;
    boolean liftControl = false;

    //speed multiplier for driver practice
    public static double drive_speed_M = 0.5;





    public void runOpMode() {

        //intialize the outtake to be ready for pixels
        target = lift_intake;
        aPos = arm_intake;
        bPosx = bucket_intake;
        outtake_lid.setPosition(out_open);
        
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        controller = new PIDController(p, i, d);

        FR = hardwareMap.get(DcMotorEx.class, "FR");
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR.setDirection(DcMotorEx.Direction.REVERSE);
        FR.setDirection(DcMotorEx.Direction.REVERSE);

        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setDirection(DcMotorEx.Direction.REVERSE);


        arm = hardwareMap.get(Servo.class, "arm");
        bucket = hardwareMap.get(Servo.class, "bucket");
        pincer_left = hardwareMap.get(Servo.class, "pincer_left");
        pincer_right = hardwareMap.get(Servo.class, "pincer_right");
        outtake_lid = hardwareMap.get(Servo.class, "outtake_lid");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        drone = hardwareMap.get(Servo.class, "drone");
        winch = hardwareMap.get(DcMotorEx.class, "winch");
        winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        
        waitForStart();

        while (opModeIsActive()) {

// drive using manual mode

            double d_power = .8 - .4 * gamepad1.left_trigger + (.5 * gamepad1.right_trigger);
            double drive = gamepad1.left_stick_y * drive_speed_M;
            double rotate = -gamepad1.right_stick_x * drive_speed_M;

            BL.setPower(drive + rotate);
            FL.setPower(drive + rotate);
            BR.setPower(drive - rotate);
            FR.setPower(drive - rotate);

            if (gamepad1.dpad_up) {
                BL.setPower(-d_power * drive_speed_M);
                FL.setPower(-d_power * drive_speed_M);
                BR.setPower(-d_power * drive_speed_M);
                FR.setPower(-d_power * drive_speed_M);
            }
            else if (gamepad1.dpad_down) {
                BL.setPower(d_power * drive_speed_M);
                FL.setPower(d_power * drive_speed_M);
                BR.setPower(d_power * drive_speed_M);
                FR.setPower(d_power * drive_speed_M);
            }
            else if (gamepad1.dpad_left) {
                BL.setPower(-d_power * drive_speed_M);
                FL.setPower(d_power * drive_speed_M);
                BR.setPower(d_power * drive_speed_M);
                FR.setPower(-d_power * drive_speed_M);
            }
            else if (gamepad1.dpad_right) {
                BL.setPower(d_power * drive_speed_M);
                FL.setPower(-d_power * drive_speed_M);
                BR.setPower(-d_power * drive_speed_M);
                FR.setPower(d_power * drive_speed_M);
            }


            //drone
            if (gamepad1.b) {
                drone.setPosition(launch);
            }

            //winch
            if (gamepad1.x) {
                wPos = winch.getCurrentPosition() + wSpeed;

            }
            if (gamepad1.y) {
                wPos = winch.getCurrentPosition() - wSpeed;
            }

            winch.setTargetPosition(wPos);
            winch.setPower(wdPower);
            winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //bucket lid to release pixels

            if (gamepad1.options) {
                outtake_lid.setPosition(out_half);
            }
            if (gamepad1.share) {
                outtake_lid.setPosition(out_open);
            }

            //DRIVER 2//

//intake
            if (gamepad2.x) {
                intake.setPower(power);
            }
            if (gamepad2.b) {
                intake.setPower(backPower);
            }
            if (gamepad2.a) {
                intake.setPower(0);
            }

//close bucket lid to secure pixels
            if (gamepad2.start) {
                outtake_lid.setPosition(out_shut);
            }

//pincers:
//one button method
            pincer_left.setPosition(left_open);
            pincer_right.setPosition(right_open);

            if (gamepad2.y) {
                resetRuntime();
                while (getRuntime() < pince_time) {
                    pincer_left.setPosition(left_closed);
                    pincer_right.setPosition(right_closed);
                }
            }

//right bumper moves the lift up a bit and moves the bucket to scoring position
//left bumper sends everything to intake positions
            if (gamepad2.right_bumper) {
                target = 700;
                liftControl = true;
            }

            if (gamepad2.left_bumper) {
                liftControl = false;
            }


            if (liftControl) {

                aPos = arm_score;
                bPosx = bucket_score;

                //manual lift controls
                if (gamepad2.left_stick_y < -0.2 || gamepad2.left_stick_y > 0.2) {
                    target += -gamepad2.left_stick_y * liftM;
                }

                //dpad buttons send the lift up to the right height
                if (gamepad2.dpad_down) {
                    target = 1100;
                }
                if (gamepad2.dpad_left) {
                    target = 2000;
                }
                if (gamepad2.dpad_up) {
                    target = 2400;
                }

                //manual bucket controls
                if (gamepad2.left_stick_button && bucket.getPosition() < 0.99) {
                    bPosx += bChange;
                }
                if (gamepad2.right_stick_button && bucket.getPosition() > 0.01) {
                    bPosx -= bChange;
                }
            }

            if (!liftControl) {
                //send the arm and bucket back to intake positions
                aPos = arm_intake;
                bPosx = bucket_intake;

                //once the arm is actually back, send the lift to intake position
             if(gamepad2.back) {
                 target = lift_intake;
             }

            }


//send the lift to the current value of the target variable
            target = Range.clip(target, 110, 2400);
            controller.setPID(p, i, d);
            int liftPos1 = lift1.getCurrentPosition();
            int liftPos2 = lift2.getCurrentPosition();
            double pid = controller.calculate(liftPos1, target);
            double pid2 = controller.calculate(liftPos2, target);
            double ff = 0;

            double lPower1 = pid + ff;
            double lPower2 = pid2 + ff;

            lift1.setPower(lPower1);
            lift2.setPower(lPower2);

//send the arm to the current value of the aPos variable
            aPos = Range.clip(aPos, .01, .99);
            arm.setPosition(aPos);

//send the bucket to the current value of the bPosx variable
            bPosx = Range.clip(bPosx, .01, .99);
            bucket.setPosition(bPosx);

//send the relevant variables to the driver station
            telemetry.addData("target", target);
            telemetry.addData("pos1", lift1.getCurrentPosition());
            telemetry.addData("power1", lift1.getPower());
            telemetry.addData("pos2", lift2.getCurrentPosition());
            telemetry.addData("power2", lift2.getPower());
            telemetry.addData("arm position", aPos);
            telemetry.addData("bucket position", bPosx);
            telemetry.addData("pincer_left", pincer_left.getPosition());
            telemetry.addData("pincer_right", pincer_right.getPosition());
            telemetry.update();
        }
    }
}