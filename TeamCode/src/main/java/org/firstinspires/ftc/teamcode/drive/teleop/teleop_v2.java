package org.firstinspires.ftc.teamcode.drive.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

@Config
@TeleOp(name = "teleop_v2", group = "teleop")
public class teleop_v2 extends LinearOpMode {

    //pid
    private PIDController controller;
    public static double p = 0.005, i = 0, d = 0;
    public static double f = 0;
    public static double target = 110;

    boolean liftControl = false;
    ArrayList<Boolean> booleanArray = new ArrayList<Boolean>();
    int booleanIncrementer = 0;
    public static double pince_time = 0.2;

    DcMotorEx FR;
    DcMotorEx FL;
    DcMotorEx BR;
    DcMotorEx BL;
    double rotate;
    //intake
    DcMotorEx intake;

    public static double power = 1;
    public static double backPower = -1;
    //lift
    DcMotorEx lift1;
    DcMotorEx lift2;

    public static double liftM = 40;
    public static double liftMax = 2000;

    //winch
    DcMotorEx winch;

    public static int wBack = -200;
    public static int wPos = 0;
    public static int wSpeed = 50;
    public static int wUp = 2000;
    public static int wDown = 1500;
    public static double wbPower = .5;
    public static double wuPower = 1;
    public static double wdPower = 1;

    //arm
    public static double aPos = .03;
    public static double bPosx = .2;
    public static double bChange = .001;
    public static double drive_slow = 0.5;



    ServoImplEx Arm;
    ServoImplEx bucket;
    Servo outtake_lid;
    // AnalogInput sEncoder;
    AnalogInput sEncoder;
    AnalogInput sEncoder2;

    //pincer
    Servo pincer_left;
    Servo pincer_right;

    //drone
    Servo drone;

    public static double launch = .3;
    public static double right_open = 0.3;
    public static double right_closed = 0.1;
    public static double left_open = 0.69;
    public static double left_closed = 0.89;
    public static double out_shut = 0;
    public static double out_half = .5;
    public static double out_open = 1;
    public static double bucket_score = 0.55;
    public static double bucket_intake = 0.13;
    public static double arm_intake = 0.13;
    public static double arm_score = 0.89;
    public static double lift_intake = 110;
    public static double lift_low = 110;



    public void runOpMode() {
        target = 110;
        aPos = arm_intake;
        bPosx = bucket_intake;

        double  drive;
        double  strafe;
        double  turn;
        double drive_speed = 1;
        
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//pid
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


        Arm = (ServoImplEx) hardwareMap.get(Servo.class, "Arm");
        bucket = (ServoImplEx) hardwareMap.get(Servo.class, "bucket");
        Arm.setPwmRange(new PwmControl.PwmRange(505, 2495));
        bucket.setPwmRange(new PwmControl.PwmRange(505, 2495));
        sEncoder = hardwareMap.get(AnalogInput.class, "sEncoder");
        sEncoder2 = hardwareMap.get(AnalogInput.class, "sEncoder2");
        pincer_left = hardwareMap.get(Servo.class, "pincer_left");
        pincer_right = hardwareMap.get(Servo.class, "pincer_right");
        outtake_lid = hardwareMap.get(Servo.class, "outtake_lid");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        drone = hardwareMap.get(Servo.class, "drone");
        winch = hardwareMap.get(DcMotorEx.class, "winch");

        waitForStart();

        while (opModeIsActive()) {

//two driving methods --> dPad strafes or left stick strafes

      /*      //dPad strafes method

            double d_power = .8 - .4 * gamepad1.left_trigger + (.5 * gamepad1.right_trigger);
            double drive = gamepad1.left_stick_y;
            double rotate_stick = -gamepad1.right_stick_x;
            double rotate_button = 0;

            rotate = rotate_stick + .5 * rotate_button;

            BL.setPower(drive + rotate);
            FL.setPower(drive + rotate);
            BR.setPower(drive - rotate);
            FR.setPower(drive - rotate);

            if (gamepad1.dpad_up) {
                BL.setPower(-d_power);
                FL.setPower(-d_power);
                BR.setPower(-d_power);
                FR.setPower(-d_power);
            } else if (gamepad1.dpad_down) {
                BL.setPower(d_power);
                FL.setPower(d_power);
                BR.setPower(d_power);
                FR.setPower(d_power);
            } else if (gamepad1.dpad_left) {
                BL.setPower(-d_power);
                FL.setPower(d_power);
                BR.setPower(d_power);
                FR.setPower(-d_power);
            } else if (gamepad1.dpad_right) {
                BL.setPower(d_power);
                FL.setPower(-d_power);
                BR.setPower(-d_power);
                FR.setPower(d_power);
            }

       */

           //left stick strafes method
            if(gamepad1.right_bumper) {
                drive_speed = drive_slow;
            }
            else {
                drive_speed = 1;
            }

            drive  = -gamepad1.left_stick_y * drive_speed;
            strafe = -gamepad1.left_stick_x * drive_speed;
            turn   = -gamepad1.right_stick_x * drive_speed;

            moveRobot(drive, strafe, turn);

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
            if (gamepad1.start) {
                outtake_lid.setPosition(out_shut);
            }
            if (gamepad1.touchpad) {
                outtake_lid.setPosition(out_half);
            }
            if (gamepad1.back) {
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

//pincers: three options --> two buttons, toggle, one button

/*//two buttons method

            if (gamepad2.right_trigger > 0.2) {
                pincer_right.setPosition(right_closed);
                pincer_left.setPosition(left_closed);
            }
            if (gamepad2.left_trigger > 0.2) {
                pincer_right.setPosition(right_open);
                pincer_left.setPosition(left_open);
            }
*/

/*//toggle method
            boolean G2YPressed = ifPressed(gamepad2.y);
            double pincerPos = pincer_left.getPosition();

            if (G2YPressed && pincerPos == left_open) {
                pincer_left.setPosition(left_closed);
                pincer_right.setPosition(right_closed);
            }
            else if (G2YPressed && pincerPos == left_closed) {
                pincer_left.setPosition(left_open);
                pincer_right.setPosition(right_open);
            }
 */

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




            double pos = sEncoder.getVoltage() / 3.3 * 360;
            double pos2 = sEncoder2.getVoltage() / 3.3 * 360;

//right bumper moves the lift up a bit and moves the bucket to scoring position
//left bumper sends everything to intake positions
            if (gamepad2.right_bumper) {
                liftControl = true;
                target = 300;
                bPosx = bucket_score;
            }
            if (gamepad2.left_bumper) {
                liftControl = false;
            }


            if (liftControl) {

                //manual lift controls
                if (gamepad2.left_stick_y < -0.2 || gamepad2.left_stick_y > 0.2) {
                    target += -gamepad2.left_stick_y * liftM;
                }

                //dpad buttons send the lift up to the right height
                //hold down the button or press again to send arm to scoring position
                if (gamepad2.dpad_down) {
                    target = 1100;
                    if (lift1.getCurrentPosition() > 400) {
                        aPos = arm_score;
                    }
                }
                if (gamepad2.dpad_left) {
                    target = 2000;
                    if (lift1.getCurrentPosition() > 400) {
                        aPos = arm_score;
                    }
                }
                if (gamepad2.dpad_up) {
                    target = 2400;
                    if (lift1.getCurrentPosition() > 400) {
                        aPos = arm_score;
                    }
                }

                //manual bucket controls
                if (gamepad2.start && bucket.getPosition() < 0.99) {
                    bPosx += bChange;
                }
                if (gamepad2.share && bucket.getPosition() > 0.01) {
                    bPosx -= bChange;
                }
            }

            if (!liftControl) {
                //send the arm and bucket back to intake positions
                aPos = arm_intake;
                bPosx = bucket_intake;
                //once the arm is actually back, send the lift to intake position
                if (Arm.getPosition() < 0.2) {
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
            Arm.setPosition(aPos);

//send the bucket to the current value of the bPosx variable
            bPosx = Range.clip(bPosx, .01, .99);
            bucket.setPosition(bPosx);

//reset the boolean incrementer for your toggle trick
            booleanIncrementer = 0;

            telemetry.addData("target", target);
            telemetry.addData("Run time", getRuntime());
            telemetry.addData("pos1", lift1.getCurrentPosition());
            telemetry.addData("power1", lift1.getPower());
            telemetry.addData("pos2", lift2.getCurrentPosition());
            telemetry.addData("power2", lift2.getPower());
            telemetry.addData("arm", pos);
            telemetry.addData("arm position", aPos);
            telemetry.addData("bucket", pos2);
            telemetry.addData("bucket position", bPosx);
            telemetry.addData("pincer_left", pincer_left.getPosition());
            telemetry.addData("pincer_right", pincer_right.getPosition());
            telemetry.update();
        }

    }

    private boolean ifPressed(boolean button) {
        boolean output = false;
        if (booleanArray.size() == booleanIncrementer) {
            booleanArray.add(false);
        }
        boolean buttonWas = booleanArray.get(booleanIncrementer);
        if (button != buttonWas && button) {
            output = true;
        }
        booleanArray.set(booleanIncrementer, button);

        booleanIncrementer += 1;
        return output;
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x - y - yaw;
        double rightFrontPower   =  x + y + yaw;
        double leftBackPower     =  x + y - yaw;
        double rightBackPower    =  x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        FL.setPower(leftFrontPower);
        FR.setPower(rightFrontPower);
        BL.setPower(leftBackPower);
        BR.setPower(rightBackPower);
    }
}