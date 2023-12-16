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

    //April Tag Setup
    public static double DESIRED_DISTANCE = 6.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    boolean targetFound = false;    // Set to true when an AprilTag target is detected
    double tagDrive = 0;        // Desired forward power/speed (-1 to +1)
    double strafe = 0;        // Desired strafe power/speed (-1 to +1)
    double turn = 0;        // Desired turning power/speed (-1 to +1)
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag


    //pid setup
    private PIDController controller;
    public static double p = 0.005, i = 0, d = 0;
    public static double f = 0;
    public static double target = 110;

    //intialize motors

    DcMotorEx FR;
    DcMotorEx FL;
    DcMotorEx BR;
    DcMotorEx BL;
    DcMotorEx intake;
    DcMotorEx lift1;
    DcMotorEx lift2;
    DcMotorEx winch;
    ServoImplEx Arm;
    ServoImplEx bucket;
    Servo outtake_lid;
    AnalogInput sEncoder;
    AnalogInput sEncoder2;
    Servo pincer_left;
    Servo pincer_right;
    Servo drone;

    //intake settings
    public static double power = 1;
    public static double backPower = -1;

    //lift settings
    public static double liftM = 40;
    public static double liftMax = 2000;

    //winch settings
    public static int wPos = 0;
    public static int wSpeed = 50;
    public static double wdPower = 1;

    //arm and bucket settings
    public static double aPos = .03;
    public static double bPosx = .2;
    public static double bChange = .001;

    //drive settings
    public static double drive_slow = 0.5;

    //pincer setup
    boolean liftControl = false;
    ArrayList<Boolean> booleanArray = new ArrayList<Boolean>();
    int booleanIncrementer = 0;
    public static double pince_time = 0.2;
    public static double right_open = 0.3;
    public static double right_closed = 0.1;
    public static double left_open = 0.69;
    public static double left_closed = 0.89;

    //drone setup
    public static double launch = .3;

    //bucket lid setup
    public static double out_shut = 0;
    public static double out_half = .5;
    public static double out_open = 1;

    //arm and bucket setup
    public static double bucket_score = 0.55;
    public static double bucket_intake = 0.13;
    public static double arm_intake = 0.13;
    public static double arm_score = 0.89;
    public static double lift_intake = 110;





    public void runOpMode() {

        // Initialize the Apriltag Detection process
        initAprilTag();

        target = 110;
        aPos = arm_intake;
        bPosx = bucket_intake;

        double  drive;
        double  strafe;
        double  turn;
        double dPad_strafe = 1;
        double dPad_drive = 1;
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

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        waitForStart();

        while (opModeIsActive()) {

            targetFound = false;
            desiredTag = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (gamepad1.left_bumper && targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else {

                // drive using manual mode, use right bumper for slow mode
                if(gamepad1.right_bumper) {
                    drive_speed = drive_slow;
                }
                else {
                    drive_speed = 1;
                }
                if (gamepad1.dpad_right) {
                    dPad_strafe = 1;
                }
                if (gamepad1.dpad_left) {
                    dPad_strafe = -1;
                }
                if (gamepad1.dpad_up) {
                    dPad_drive = 1;
                }
                if (gamepad1.dpad_down) {
                    dPad_drive = -1;
                }
                if(gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) {
                    dPad_drive = -gamepad1.left_stick_y;
                }

                drive  = dPad_drive * drive_speed;
                strafe = dPad_strafe * drive_speed;
                turn   = -gamepad1.right_stick_x * drive_speed;

            }

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
}