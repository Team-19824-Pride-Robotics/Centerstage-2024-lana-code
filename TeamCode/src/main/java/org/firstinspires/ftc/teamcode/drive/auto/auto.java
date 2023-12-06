package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name="auto", group="auto")

//@Disabled
public class auto extends LinearOpMode {

    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));


    @Override
    public void runOpMode() throws InterruptedException {

lift lift = new lift(hardwareMap);
SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(startPose)
                .forward(10)
                .addDisplacementMarker(lift::top)
                .strafeRight(10)
                .addDisplacementMarker(lift::zero)
                .build();

        drive.followTrajectorySequenceAsync(trajectory);

        while (!isStarted()) {


        }

        waitForStart();

        if (opModeIsActive()) {

            drive.update();
            lift.update();

            if (!isStopRequested()) {
            }


        }
    }


}