package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "a", name="2 Specimen ")
public class two_specimen extends LinearOpMode {
    public DcMotor elevator = null;
    public Servo claw = null;



    public void runOpMode() {
        elevator = hardwareMap.get(DcMotor.class,"elevator_motor");
        claw =  hardwareMap.get(Servo.class, "claw");
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
                .splineToConstantHeading(new Vector2d(0, -31.5), Math.toRadians(34.39),

                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
                .build();
        //to the parking zone
        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(new Pose2d(-1.41, -34.18, Math.toRadians(90.00)))
                .lineTo(new Vector2d(45.90, -55))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))  // Velocity constraint
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))  // Acceleration constraint
                .build();


// back to submerisible
        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(new Pose2d(45.90, -50, Math.toRadians(270.00)))
                .lineTo(new Vector2d(0.37, -33.59))  // Move to the new point
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))  // Slower velocity
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))  // Reduced acceleration
                .build();


          // back a little after 1
        TrajectorySequence backalittle = drive.trajectorySequenceBuilder(trajectory1.end())
                .back(7)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))  // Slower velocity
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))  // Reduced acceleration
                .build();


      //back a little after 2
        TrajectorySequence backalittle2 = drive.trajectorySequenceBuilder(trajectory2.end())
                .back(7)
        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))  // Slower velocity
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))  // Reduced acceleration
                .build();





        elevator.setTargetPosition(2000);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.6);

        drive.followTrajectorySequence(trajectory0);
        elevator.setTargetPosition(1500);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.9);

        sleep(2000);

        claw.setPosition(0.7);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.9);

        sleep(1000);
        drive.followTrajectorySequence(trajectory1);
        sleep(1000);


        drive.followTrajectorySequence(backalittle);
        sleep(1000);
        claw.setPosition(1);

        sleep(2000);
         elevator.setTargetPosition(2100);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.6);
        sleep(1000);
        drive.followTrajectorySequence(trajectory2);
        sleep(1000);
        drive.followTrajectorySequence(backalittle2);
        elevator.setTargetPosition(1500);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.9);

        sleep(2000);

        claw.setPosition(0.7);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.9);

        drive.followTrajectorySequence(trajectory1);

        drive.followTrajectorySequence(backalittle);







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



