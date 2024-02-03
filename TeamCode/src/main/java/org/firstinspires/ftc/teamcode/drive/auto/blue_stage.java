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
@Autonomous(name="blue_stage ", group="auto")
public class blue_stage extends OpMode
{

    public static double x1 = -30;
    public static double y1 = -5;
    public static double h1 = 0;
    public static double ht1 = -10;
    public static double x2 = -35;
    public static double y2 = -34;
    public static double h2 = -78;
    public static double ht2 = 0;
    public static double x3 = -40;
    public static double y3 = -45;
    public static double h3 = -90;
    public static double ht3 = 0;


    DistanceSensor distance2;
    DistanceSensor distance4;

    public SampleMecanumDrive drive;

    DcMotor intake;
    DcMotorEx lift1;
    DcMotorEx lift2;
    ServoImplEx arm;
    Servo outtake_lid;
    Servo bucket;
    // AnalogInput sEncoder;
    AnalogInput arm_encoder;
    //AnalogInput sEncoder2;

    public static int a = 500;
    public static int b = 100;

    //pid
    private PIDController controller;
    public static double p = 0.005, i = 0, d =0;
    public static double f = 0;
    public static double target = 0;

    double liftpos1;
    double liftpos2;

    double aPos =.15;

    public static double bPosx =.35;
    public static double score = 250;
    double test = 0;

    @Override
    public void init() {
        target = 0;
        bPosx = 0.40;
        aPos = 0.88;
        controller = new PIDController(p,i,d);

         drive = new SampleMecanumDrive(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");

      lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
      lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
       lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setDirection(DcMotorEx.Direction.REVERSE);


        arm = (ServoImplEx) hardwareMap.get(Servo.class, "arm");
        bucket = hardwareMap.get(Servo.class, "bucket");
        outtake_lid = hardwareMap.get(Servo.class, "outtake_lid");
        arm.setPwmRange(new PwmControl.PwmRange(505, 2495));
        //bucket.setPwmRange(new PwmControl.PwmRange(505, 2495));
        arm_encoder = hardwareMap.get(AnalogInput.class, "arm_encoder");
        //sEncoder2 = hardwareMap.get(AnalogInput.class, "sEncoder2");

        distance2 = hardwareMap.get(DistanceSensor.class, "distance2");
        distance4 = hardwareMap.get(DistanceSensor.class, "distance4");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");


    }

    @Override
    public void init_loop() {

        if (distance2.getDistance(DistanceUnit.CM)<200) {
            x1 = -24;
            y1 = -5;
            h1 = 25;

            x2 = -14.5;
            y2= -33;
        }
        else if (distance4.getDistance(DistanceUnit.CM)<200) {
            x1 = -24;
            y1 = 5;
            h1 = 0;

            x2 = -22;
            y2=-33;
        }
        else {
            x1 = -23;
            y1 = 8;
            h1 = -45;

            x2 = -27;
            y2 =-34;
        }

        telemetry.addData("distance4", distance4.getDistance(DistanceUnit.CM));
        telemetry.addData("distance2", distance2.getDistance(DistanceUnit.CM));
        telemetry.update();
    }

    @Override
    public void start() {

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    target = 500;
                })

                //drive to the middle of the spike marks and point the intake at the correct one
                .lineToLinearHeading(new Pose2d(x1, y1, Math.toRadians(h1)))

                //turn the intake on for long enough to spit out the purple pixel
                .addTemporalMarker(() -> intake.setPower(-0.6))
                .waitSeconds(1)
                .addTemporalMarker(() -> intake.setPower(0))

                //back up a bit to make sure you don't hit the pixel
                .forward(4.5)

                //raise the lift and move the arm and bucket into position
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    aPos = 0.45;
                    bPosx = 0.214;
                })
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    target = score;
                })

                //drive over to the backdrop with the lift facing it
                .lineToLinearHeading(new Pose2d(x2, y2, Math.toRadians(h2)))
                .waitSeconds(0.5)

                //open the door to score the pixel
                .addTemporalMarker(() -> {
                    outtake_lid.setPosition(0.01);
                })

                //wait for the pixel to get scored
                .waitSeconds(2)

                //move out of the way in case the other team needs to get there
                .back(8)

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

                .lineToLinearHeading(new Pose2d(x3, y3, Math.toRadians(h3)))

                .build();

        drive.followTrajectorySequenceAsync(trajSeq);
    }

    @Override
    public void loop() {

          drive.update();

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

        liftpos1 = lift1.getCurrentPosition();
        liftpos2 = lift2.getCurrentPosition();

        arm.setPosition(aPos);
        bucket.setPosition(bPosx);
       // intake.update();
        double pos = arm_encoder.getVoltage() / 3.3 * 360;
        //double pos2 = sEncoder2.getVoltage() / 3.3 * 360;

        telemetry.addData("liftpos1", liftpos1);
        telemetry.addData("liftpos1", liftpos1);
        telemetry.addData("arm", pos);
        telemetry.addData("test", test);
        telemetry.addData("heading", drive.getPoseEstimate());
        //telemetry.addData("bucket", pos2);
        telemetry.update();
    }

    @Override
    public void stop() {

    }


}
