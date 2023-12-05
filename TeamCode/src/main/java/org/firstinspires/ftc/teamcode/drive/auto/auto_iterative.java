package org.firstinspires.ftc.teamcode.drive.auto;

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

@Autonomous(name="auto_iterative", group="auto")
@Disabled

public class auto_iterative extends OpMode
{
    arm arm = new arm(hardwareMap);
    lift lift = new lift(hardwareMap);
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));





    int teamProp;

    @Override
    public void init() {

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(startPose)
                .forward(10)
                .addDisplacementMarker(() -> {
                    lift.top();
                })
                .strafeRight(10)
                .addDisplacementMarker(() -> {
                    lift.zero();
                })
                .build();

        drive.followTrajectorySequenceAsync(trajectory);

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        drive.update();
        lift.update();
    }

    @Override
    public void stop() {

    }


}
