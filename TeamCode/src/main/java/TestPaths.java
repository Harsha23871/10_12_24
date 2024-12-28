//package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "a", name="TestTrajects")
public class TestPaths extends LinearOpMode {
    public DcMotor elevator = null;
    public Servo claw,wrist = null;



    public void runOpMode() {
        elevator = hardwareMap.get(DcMotor.class,"elevator_motor");
        claw =  hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


//        hwMapForAuto.elevator_Scoring_Pos();  // Moves elevator to scoring position
//        hwMapForAuto.elevator_Resting_Pos();
        //elevator = hardwareMap.get(DcMotor.class,"elevator_motor");
        claw.setDirection(Servo.Direction.FORWARD);
        claw.setPosition(1);

        waitForStart();

//        elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        // Moves elevator to resting position

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(11,  -61, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        // to the submersible
        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(11, -61, Math.toRadians(270.00)))
                .splineToConstantHeading(new Vector2d(0, -29), Math.toRadians(34.39),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        //to the parking zone

        TrajectorySequence TrajectoryFarCurve = drive.trajectorySequenceBuilder(new Pose2d(0.00, -29.00, Math.toRadians(270.00)))
                .lineToConstantHeading(new Vector2d(34.33, -47.98))
                .splineTo(new Vector2d(47.38, -3.78), Math.toRadians(55.00))
                .build();

        TrajectorySequence trajectory1ststep = drive.trajectorySequenceBuilder(new Pose2d(0.00, -29.00, Math.toRadians(270.00)))
                .lineTo(new Vector2d(26, -51.24))
                .splineTo(new Vector2d(43.67, -11.35), Math.toRadians(0))

                .build();

       // Use arm to pickup speciment then put them in zone all at once then score
        // back a little after trajectory 1

















        // slow down first one because inertia



//               .lineToLinearHeading(new Pose2d(37.00, -41.45, Math.toRadians(270.00)))
//               .splineTo(new Vector2d(36.41, -12), Math.toRadians(90.94))
//                .lineTo(new Vector2d(48.57, -12))
//                .lineTo(new Vector2d(47.98, -58.65))

//               .lineTo(new Vector2d(36.11, -45.31))
//               .lineTo(new Vector2d(37.45, -8.23))
//               .lineTo(new Vector2d(47.68, -4.82))
//               .lineTo(new Vector2d(48.42, -55.98))



//        wrist.setPosition(0);
//        elevator.setTargetPosition(2000);                  //FIRST SPECIMEN
//        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        elevator.setPower(0.6);
        drive.followTrajectorySequence(trajectory0);
//        elevator.setTargetPosition(1400);
//        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        elevator.setPower(0.6);
        sleep(500);
//        claw.setPosition(0.7);
//        elevator.setTargetPosition(0);
//        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        elevator.setPower(0.6);
        drive.followTrajectorySequence(trajectory1ststep);





        //     claw position 1 is close
        //0.7 is open
        //RETURN TO WALL AND INTAKE
      //  drive.followTrajectorySequence(FirstPush);

















// 35.81,-37.15 start pos
        if (isStopRequested()) return;
        //robot.hwMap();

//        sleep(2000);









        /*TrajectorySequence trajectory7 = drive.trajectorySequenceBuilder(new Pose2d(11, -62, Math.toRadians(90.00)))
                .lineTo(new Vector2d(51.83, -62.36)) //Red side observation Auto( Preload into Observation zone)
                .lineTo(new Vector2d(10.90, -61.03))
                .build();*/








               /*
                TrajectorySequence trajectory7 = drive.trajectorySequenceBuilder(new Pose2d(11, -62, Math.toRadians(90.00)))
                .lineTo(new Vector2d(51.83, -62.36)) //Red side observation Auto( Preload into Observation zone)
                .lineTo(new Vector2d(10.90, -61.03))
                .build();*/










                /*.splineTo(new Vector2d(-49.46, -26.18), Math.toRadians(111.36))



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


    }   }




