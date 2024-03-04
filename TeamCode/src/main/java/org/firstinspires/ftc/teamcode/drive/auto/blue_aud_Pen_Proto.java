package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name="blue_aud_Pen", group="auto")
public class blue_aud_Pen_Proto extends OpMode
{

    public static double x1 = -30;
    public static double y1 = -5;
    public static double h1 = 0;
    public static double x3 = -50;
    public static double y3 = 20;
    public static double h3 = -95;
    public static double x4 = -28;
    public static double y4 = 20;
    public static double h4 = -95;
    public static double x2 = -35;
    public static double y2 = -75;
    public static double h2 = -75;
    double aPos =.15;
    public static double bPosx =.35;
    public static double score = 250;
    private PIDController controller;
    public static double p = 0.005, i = 0, d =0;
    public static double f = 0;
    public static double target = 0;


    DistanceSensor distance1;
    DistanceSensor distance3;
    DcMotorEx lift1;
    DcMotorEx lift2;
    ServoImplEx arm;
    Servo outtake_lid;
    Servo bucket;
    Servo PinL;
    Servo PinR;
    // AnalogInput sEncoder;
    AnalogInput arm_encoder;

    public SampleMecanumDrive drive;

    DcMotorEx intake;


    @Override
    public void init() {

        target = 0;
        bPosx = 0.40;
        aPos = 0.88;
        controller = new PIDController(p,i,d);


        drive = new SampleMecanumDrive(hardwareMap);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setDirection(DcMotorEx.Direction.REVERSE);
        PinL = hardwareMap.get(Servo.class, "pincer_left");
        PinR = hardwareMap.get(Servo.class, "pincer_right");
        arm = (ServoImplEx) hardwareMap.get(Servo.class, "arm");
        bucket = hardwareMap.get(Servo.class, "bucket");
        outtake_lid = hardwareMap.get(Servo.class, "outtake_lid");
        arm.setPwmRange(new PwmControl.PwmRange(505, 2495));
        //bucket.setPwmRange(new PwmControl.PwmRange(505, 2495));
        arm_encoder = hardwareMap.get(AnalogInput.class, "arm_encoder");
        //sEncoder2 = hardwareMap.get(AnalogInput.class, "sEncoder2");

        distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
        distance3 = hardwareMap.get(DistanceSensor.class, "distance3");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");


    }

    @Override
    public void init_loop() {

        if (distance1.getDistance(DistanceUnit.CM)<200) {
            x1 = -24;
            y1 = 5;
            h1 = -25;

            x2 = -15;

        }
        else if (distance3.getDistance(DistanceUnit.CM)<200) {
            x1 = -24;
            y1 = -5;
            h1 = 0;

            x2 = -22;

        }
        else {
            x1 = -23;
            y1 = -8;
            h1 = 45;

            x2 = -28;

        }

        telemetry.addData("distance3", distance3.getDistance(DistanceUnit.CM));
        telemetry.addData("distance1", distance1.getDistance(DistanceUnit.CM));
        telemetry.update();
    }

    @Override
    public void start() {

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .back(10)
                //drive to the middle of the spike marks and point the intake at the correct one
                .lineToLinearHeading(new Pose2d(x1, y1, Math.toRadians(h1)))
                //turn the intake on for long enough to spit out the purple pixel
                //.addTemporalMarker(() -> intake.setPower(-0.60))
                .addTemporalMarker(() -> PinR.setPosition(.93))
                .addTemporalMarker(() -> PinL.setPosition(0.71))

                //.waitSeconds(1)
                //.addTemporalMarker(() -> intake.setPower(0))
                //back up a bit to make sure you don't hit the pixel
                .forward(5)
/*
                //set up for traversing through the stage door
                .lineToLinearHeading(new Pose2d(x3, y3, Math.toRadians(h3)))

                //drive under the stage door
                .lineToLinearHeading(new Pose2d(x3, -40, Math.toRadians(h3)))


                //raise the lift and move the arm and bucket into position
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    target = 500;
                })
                .addTemporalMarker(() -> {
                    aPos = 0.45;
                    bPosx = 0.214;

                })
                .waitSeconds(.5)

                //move the lift down to scoring position
                .addTemporalMarker(() -> {
                    target = score;
                })

                //drive over to the backdrop with the lift facing it
                .splineToLinearHeading(new Pose2d(x2, y2, Math.toRadians(h2)), Math.toRadians(0))
                .waitSeconds(0.5)

                //open the door to score the pixel
                .addTemporalMarker(() -> {
                    outtake_lid.setPosition(0.6);
                })

                //wait for the pixel to get scored
                .waitSeconds(2)

                //move out of the way in case the other team needs to get there
                .back(6)

                //raise the lift back up to get the arm back in
                .addTemporalMarker(() -> {
                    target = 500;
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    bPosx = 0.4;
                })
                .addTemporalMarker(() -> {
                    aPos = 0.88;
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    target = 0;
                })
                .waitSeconds(1)

                .lineToLinearHeading(new Pose2d(-40, y4, Math.toRadians(h3)))



 */


                .build();

        drive.followTrajectorySequenceAsync(trajSeq);
    }

    @Override
    public void loop() {

          drive.update();


        telemetry.update();
    }

    @Override
    public void stop() {

    }


}
