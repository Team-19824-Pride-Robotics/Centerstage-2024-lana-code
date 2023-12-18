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

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

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

    //drive settings
    public static double drive_slow = 0.5;

    //pincer setup
    boolean liftControl = false;
    ArrayList<Boolean> booleanArray = new ArrayList<Boolean>();
    int booleanIncrementer = 0;
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





    public void runOpMode() {

        // Initialize the Apriltag Detection process
       // initAprilTag();

        target = lift_intake;
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


        arm = hardwareMap.get(Servo.class, "arm");
        bucket = hardwareMap.get(Servo.class, "bucket");

        pincer_left = hardwareMap.get(Servo.class, "pincer_left");
        pincer_right = hardwareMap.get(Servo.class, "pincer_right");
        outtake_lid = hardwareMap.get(Servo.class, "outtake_lid");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        drone = hardwareMap.get(Servo.class, "drone");
        winch = hardwareMap.get(DcMotorEx.class, "winch");


        outtake_lid.setPosition(out_open);

        waitForStart();

        while (opModeIsActive()) {

// drive using manual mode, use right bumper for slow mode

                if(gamepad1.right_bumper) {
                    drive_speed = drive_slow;
                }
                else {
                    drive_speed = 1;
                }

                drive  = -gamepad1.left_stick_y * drive_speed;
                strafe = -gamepad1.left_stick_x * drive_speed;
                turn   = -gamepad1.right_stick_x * drive_speed;


            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            sleep(10);


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
                liftControl = true;
            }

            if (gamepad2.left_bumper) {
                liftControl = false;
            }


            if (liftControl) {

                target = 700;
                aPos = arm_score;
                bPosx = bucket_score;

                //manual lift controls
                if (gamepad2.left_stick_y < -0.2 || gamepad2.left_stick_y > 0.2) {
                    target += -gamepad2.left_stick_y * liftM;
                }

                //dpad buttons send the lift up to the right height
                //hold down the button or press again to send arm to scoring position
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

//reset the boolean incrementer for your toggle trick
            booleanIncrementer = 0;

            telemetry.addData("target", target);
            telemetry.addData("Run time", getRuntime());
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

    /*
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void setManualExposure ( int exposureMS, int gain){
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }

    }

     */
}