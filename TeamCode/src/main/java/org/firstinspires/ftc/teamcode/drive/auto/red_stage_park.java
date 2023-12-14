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
@Autonomous(name="red_stage_park", group="auto")
public class red_stage_park extends OpMode
{

    public static double x1 = -30;
    public static double y1 = 5;
    public static double h1 = 0;
    public static double ht1 = -10;
    public static double x2 = -20;
    public static double y2 = 34;
    public static double h2 = 85;
    public static double ht2 = 0;
    public static double x3 = -45;
    public static double y3 = 33;
    public static double h3 = 86;
    public static double ht3 = -10;


    DistanceSensor distance1;
    DistanceSensor distance3;

    public SampleMecanumDrive drive;

    DcMotorEx intake;


    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
        distance3 = hardwareMap.get(DistanceSensor.class, "distance3");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");


    }

    @Override
    public void init_loop() {

        if (distance1.getDistance(DistanceUnit.CM)<200) {
            x1 = -22;
            y1 = 9;
            h1 = 0;

            x2 = -16;
        }
        else if (distance3.getDistance(DistanceUnit.CM)<200) {
            x1 = -25.5;
            y1 = -5;
            h1 = 0;

            x2 = -23;
        }
        else {
            x1 = -20.5;
            y1 = -9.5;
            h1 = 45;

            x2 = -31;
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
                .addTemporalMarker(() -> intake.setPower(0.75))
                .waitSeconds(.5)
                .addTemporalMarker(() -> intake.setPower(0))
                .waitSeconds(.5)
                //back up a bit to make sure you don't hit the pixel
                .forward(5)
                //strafe back towards the wall and then forward some to go around the scored pixel
                //drive over to the backdrop with the lift facing it
                .lineToLinearHeading(new Pose2d(x3, y3, Math.toRadians(h3)))


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
