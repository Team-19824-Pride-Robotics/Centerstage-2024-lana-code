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
@Autonomous(name="parkTest", group="auto")
public class parkTest extends OpMode
{

    public static double x1 = -22;
    public static double y1 = -10;
    public static double h1 = 0;
    public static double x2 = -20;
    public static double y2 = -40;
    public static double h2 = 0;3
    public static double x3 = -20;
    public static double y3 = -40;
    public static double h3 = 0;
    public static double x4 = -20;
    public static double y4 = -40;
    public static double h4 = 0;

    public SampleMecanumDrive drive;




    @Override
    public void init() {


         drive = new SampleMecanumDrive(hardwareMap);



        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");


    }

    @Override
    public void init_loop() {


    }

    @Override
    public void start() {

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .back(10)
                .lineToLinearHeading(new Pose2d(x1, y1, Math.toRadians(h1)))
                .lineToLinearHeading(new Pose2d(x2, y2, Math.toRadians(h2)))
                .forward(10)
                .lineToLinearHeading(new Pose2d(x3, y3, Math.toRadians(h3)))
                .back(20)
                .lineToLinearHeading(new Pose2d(x4, y4, Math.toRadians(h4)))



                .build();

        drive.followTrajectorySequenceAsync(trajSeq);
    }

    @Override
    public void loop() {

          drive.update();

    }

    @Override
    public void stop() {

    }


}
