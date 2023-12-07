package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name="blue", group="auto")
public class blue extends OpMode
{

    public static double x1 = -30;
    public static double y1 = -5;
    public static double h1 = 0;
    public static double ht1 = -10;
    public static double x2 = -20;
    public static double y2 = -40;
    public static double h2 = 270;
    public static double ht2 = 0;
    public static double x3 = -5;
    public static double y3 = -40;
    public static double h3 = 270;
    public static double ht3 = -10;


    DistanceSensor distance2;
    DistanceSensor distance4;

    public SampleMecanumDrive drive;

    public lift lift;
    //public intake intake;
    DcMotorEx intake;

    public static int a = 500;
    public static int b = 100;

    @Override
    public void init() {

        //intake = new intake(hardwareMap);
         //lift = new lift(hardwareMap);
         drive = new SampleMecanumDrive(hardwareMap);
        //arm arm = new arm(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        distance2 = hardwareMap.get(DistanceSensor.class, "distance2");
        distance4 = hardwareMap.get(DistanceSensor.class, "distance4");

        telemetry.addData("Status", "Initialized");


    }

    @Override
    public void init_loop() {

        if (distance2.getDistance(DistanceUnit.CM)<200) {
            x1 = -30;
            y1 = -5;
            h1 = 90;

            x3 = -5;
            y3 = -40;
            h3 = 270;
        }
        else if (distance4.getDistance(DistanceUnit.CM)<200) {
            x1 = -30;
            y1 = -5;
            h1 = 0;

            x3 = -5;
            y3 = -40;
            h3 = 270;
        }
        else {
            x1 = -30;
            y1 = -5;
            h1 = 0;

            x3 = -5;
            y3 = -40;
            h3 = 270;
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
                //drive to the middle of the spike marks and point the intake at the correct one
                .lineToLinearHeading(new Pose2d(x1, y1, Math.toRadians(h1)))
                //back up enough to get the intake to spit it out on the spike tape
                .forward(6)
                //turn the intake on for long enough to spit out the purple pixel
                .addTemporalMarker(() -> intake.setPower(0.75))
                .forward(1)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> intake.setPower(0))
                //back up a bit to make sure you don't hit the pixel
                .forward(2)
                //strafe back towards the wall and then forward some to go around the scored pixel
                .strafeRight(24)
                .back(24)
                //drive over to the backdrop with the lift facing it
                .splineToLinearHeading(new Pose2d(x2, y2, Math.toRadians(h2)), Math.toRadians(ht2))
                .lineToLinearHeading(new Pose2d(x3, y3, Math.toRadians(h3)))

                .build();

        drive.followTrajectorySequenceAsync(trajSeq);
    }

    @Override
    public void loop() {

          drive.update();
//        lift.update();
//        intake.update();

        telemetry.addData("liftpos1", lift.liftpos1);
        telemetry.addData("liftpos1", lift.liftpos1);
        telemetry.update();
    }

    @Override
    public void stop() {

    }


}
