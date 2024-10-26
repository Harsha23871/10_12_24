package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "a", name="Test RR auto")
public class Trajectory_sigma extends LinearOpMode {

    @Override

// 12 -58

    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36, -58, Math.toRadians(90)); // =36, -59
        drive.setPoseEstimate(startPose);


        TrajectorySequence trajectory7 = drive.trajectorySequenceBuilder(new Pose2d(-36.41, -59.99, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-40.86, -45.60), Math.toRadians(107.19))
                .splineTo(new Vector2d(-44.42, -36.11), Math.toRadians(110.56))
                .splineTo(new Vector2d(-49.31, -25.73), Math.toRadians(115.24))
                .splineTo(new Vector2d(-51.68, -30.33), Math.toRadians(242.70))
                .splineTo(new Vector2d(-55.84, -38.48), Math.toRadians(243.02))
                .splineTo(new Vector2d(-59.84, -49.16), Math.toRadians(249.44))
                .splineTo(new Vector2d(-61.92, -60.73), Math.toRadians(259.82))
                .build();


     //   Trajectory myTrajectory = drive.trajectoryBuilder(Traj1.end())
     //           .splineTo(new Vector2d(48 , -48), 0)
      //          .build();
        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(trajectory7);

      /*  Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();*/

     //   drive.followTrajectory(myTrajectory);
//splineTo(new Vector2d(x1, y1), heading)
         //       .splineTo(new Vector2d(x2, y2), heading)
         //       .build();

// -12 -48

       /* SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12, -62,Math.toRadians(270));
        TrajectorySequence middleSpike = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(12,-62))
//                .turn(Math.toRadians(45)) // Turns 45 degrees counter-clockwise
                .build();


        waitForStart();
        drive.followTrajectorySequence(middleSpike);//might need to change*/


    }



}
