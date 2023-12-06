package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.auto.lift;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name="auto_iterative", group="auto")
public class auto_iterative extends OpMode
{

    public SampleMecanumDrive drive;

    public lift lift;

    public static int a = 500;
    public static int b = 100;

    @Override
    public void init() {

         lift = new lift(hardwareMap);
         drive = new SampleMecanumDrive(hardwareMap);
        //arm arm = new arm(hardwareMap);


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
                .forward(10)
                .addDisplacementMarker(() -> {
                    lift.target = a;
                })
                .strafeRight(10)
                .addDisplacementMarker(() -> {
                    lift.target = b;
                })
                .build();

        drive.followTrajectorySequenceAsync(trajSeq);
    }

    @Override
    public void loop() {

        drive.update();
        lift.update();

        telemetry.addData("liftpos1", lift.liftpos1);
        telemetry.addData("liftpos1", lift.liftpos1);
        telemetry.update();
    }

    @Override
    public void stop() {

    }


}
