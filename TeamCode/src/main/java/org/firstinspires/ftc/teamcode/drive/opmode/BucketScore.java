package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "a", name="Bucket Score")
public class BucketScore extends LinearOpMode {

    @Override

// 12 -5

    public void runOpMode() {
        waitForStart();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-37, -61, Math.toRadians(270));

        drive.setPoseEstimate(startPose);
// try 0
        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(-37, -61, Math.toRadians(225)))
                .splineToConstantHeading(new Vector2d(-54, -53), Math.toRadians(34.39))
                .build();



        drive.followTrajectorySequence(trajectory0);









        /*TrajectorySequence trajectory7 = drive.trajectorySequenceBuilder(new Pose2d(11, -62, Math.toRadians(90.00)))
                .lineTo(new Vector2d(51.83, -62.36)) //Red side observation Auto( Preload into Observation zone)
                .lineTo(new Vector2d(10.90, -61.03))
                .build();*/


    }

}