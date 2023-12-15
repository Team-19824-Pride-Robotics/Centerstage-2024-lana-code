package org.firstinspires.ftc.teamcode.drive.teleop;

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

import java.util.List;
import java.util.concurrent.TimeUnit;


@TeleOp(name="teleop_with_AprilTag", group = "teleop")
//@Disabled

public class teleop_with_AprilTag extends LinearOpMode {

    //AprilTag Stuff

     double DESIRED_DISTANCE = 6.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    DcMotorEx FR;
    DcMotorEx FL;
    DcMotorEx BR;
    DcMotorEx BL;

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    //Our Stuff
    public PIDController controller;
    public static double p = 0.005, i = 0, d = 0;
    public static double f = 0;
    public static double target = 110;

    boolean liftControl = false;

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

    double aPos = .03;

    public static double bPosx = .2;
    public static double bChange = .001;
    ServoImplEx Arm;
    ServoImplEx bucket;
    // AnalogInput sEncoder;
    AnalogInput sEncoder;
    AnalogInput sEncoder2;

    //drone
    Servo drone;
    public static double launch = .3;
    boolean targetFound = false;    // Set to true when an AprilTag target is detected
    double tagDrive = 0;        // Desired forward power/speed (-1 to +1)
    double strafe = 0;        // Desired strafe power/speed (-1 to +1)
    double turn = 0;        // Desired turning power/speed (-1 to +1)

    @Override
    public void runOpMode() {


        // Initialize the Apriltag Detection process
        initAprilTag();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        FL = hardwareMap.get(DcMotorEx.class, "FR");
        FR = hardwareMap.get(DcMotorEx.class, "FL");
        BL = hardwareMap.get(DcMotorEx.class, "BR");
        BR = hardwareMap.get(DcMotorEx.class, "BL");
        BR.setDirection(DcMotorEx.Direction.REVERSE);
        FR.setDirection(DcMotorEx.Direction.REVERSE);

        FR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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


        intake = hardwareMap.get(DcMotorEx.class, "intake");

        drone = hardwareMap.get(Servo.class, "drone");

        winch = hardwareMap.get(DcMotorEx.class, "winch");

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();


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

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
        }


        if (gamepad1.left_bumper && targetFound) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double headingError = desiredTag.ftcPose.bearing;
            double yawError = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            tagDrive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", tagDrive, strafe, turn);


            double leftFrontPower = tagDrive - strafe - turn;
            double rightFrontPower = tagDrive + strafe + turn;
            double leftBackPower = tagDrive + strafe - turn;
            double rightBackPower = tagDrive - strafe + turn;

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
            //Don't use MoveRobot, just do what it does here. Use tagDrive instead of Drive to avoid conflict
            //make sure motor names are correct

        } else {

            target = Range.clip(target, 110, 2400);

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

          //  if (gamepad1.start) {
          //      DESIRED_DISTANCE += 0.5;
           // }

           // if (gamepad1.back) {
           //     DESIRED_DISTANCE -= 0.5;
            //}

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
            if (gamepad2.a) {
                intake.setPower(0);
            }

            double pos = sEncoder.getVoltage() / 3.3 * 360;
            double pos2 = sEncoder2.getVoltage() / 3.3 * 360;

            aPos = Range.clip(aPos, .01, .99);

            if (gamepad2.right_bumper) {
                liftControl = true;
                target = 300;
                bPosx = .45;
            }
            if (gamepad2.left_bumper) {
                liftControl = false;
                bPosx = .13;
            }


            if (liftControl) {
                if (gamepad2.y) {
                    aPos = .99;
                }
                if (gamepad2.left_stick_y < .2 || gamepad2.left_stick_y > .2) {
                    target = -gamepad2.left_stick_y * liftM + target;
                }
                if (gamepad1.start) {
                    bPosx = .9;
                }
                if (gamepad1.share) {
                    bPosx = .55;
                }
                if (gamepad2.dpad_down) {
                    target = 1100;
                }
                if (gamepad2.dpad_left) {
                    target = 2000;
                }
                if (gamepad2.dpad_up) {
                    target = 2400;
                }
                if (gamepad2.start && bucket.getPosition() < 0.99) {
                    bPosx = bucket.getPosition() + bChange;
                }
                if (gamepad2.share && bucket.getPosition() > 0.01) {
                    bPosx = bucket.getPosition() - bChange;
                }
            }

            if (!liftControl) {
                if (pos2 >= 48 && pos2 <= 52) {
                    aPos = .03;
                }
                if (pos >= 14 && pos <= 18) {
                    target = 110;
                }
            }

            controller.setPID(p, i, d);
            int liftPos1 = lift1.getCurrentPosition();
            int liftPos2 = lift2.getCurrentPosition();
            double pid = controller.calculate(liftPos1, target);
            double pid2 = controller.calculate(liftPos2, target);
            double ff = 0;

            double lPower1 = pid + ff;
            double lPower2 = pid2 + ff;

            //TEMPORARY FOR TESTING - USE PID IN ACTUAL
            //double lPower1 = .2;
            //double lPower2 = .2;

            lift1.setPower(lPower1);
            lift2.setPower(lPower2);
            Arm.setPosition(aPos);
            bucket.setPosition(bPosx);

            telemetry.addData("target", target);
            telemetry.addData("Run time", getRuntime());
            telemetry.addData("1", "test");
            telemetry.addData("pos1", lift1.getCurrentPosition());
            telemetry.addData("power1", lift1.getPower());
            telemetry.addData("pos2", lift2.getCurrentPosition());
            telemetry.addData("power2", lift2.getPower());
            telemetry.addData("arm", pos);
            telemetry.addData("bucket", pos2);
            telemetry.update();
        }

     }
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




