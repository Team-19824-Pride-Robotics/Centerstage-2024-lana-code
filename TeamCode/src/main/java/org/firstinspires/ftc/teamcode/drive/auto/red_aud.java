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
@Autonomous(name="red_aud", group="auto")
public class red_aud extends OpMode
{

    public static double x1 = -30;
    public static double y1 = 5;
    public static double h1 = 0;



    DistanceSensor distance2;
    DistanceSensor distance4;

    public SampleMecanumDrive drive;

    DcMotorEx intake;








    @Override
    public void init() {


         drive = new SampleMecanumDrive(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "intake");


        distance2 = hardwareMap.get(DistanceSensor.class, "distance2");
        distance4 = hardwareMap.get(DistanceSensor.class, "distance4");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");


    }

    @Override
    public void init_loop() {

        if (distance2.getDistance(DistanceUnit.CM)<200) {
            x1 = -25;
            y1 = -5;
            h1 = 25;

        }
        else if (distance4.getDistance(DistanceUnit.CM)<200) {
            x1 = -24;
            y1 = 5;
            h1 = 0;

        }
        else {
            x1 = -25.5;
            y1 = 5.5;
            h1 = -45;

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
                .back(10)
                //drive to the middle of the spike marks and point the intake at the correct one
                .lineToLinearHeading(new Pose2d(x1, y1, Math.toRadians(h1)))
                //turn the intake on for long enough to spit out the purple pixel
                .addTemporalMarker(() -> intake.setPower(-.6))
                .waitSeconds(1)
                .addTemporalMarker(() -> intake.setPower(0))
                //back up a bit to make sure you don't hit the pixel
                .forward(5)
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
