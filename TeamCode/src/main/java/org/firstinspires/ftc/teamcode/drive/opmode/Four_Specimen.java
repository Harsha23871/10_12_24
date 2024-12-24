package org.firstinspires.ftc.teamcode.drive.opmode;

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

@Autonomous(group = "a", name="4 Specimen Test ")
public class Four_Specimen extends LinearOpMode {
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
                .splineToConstantHeading(new Vector2d(0, -30), Math.toRadians(34.39),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        //to the parking zone

        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(new Pose2d(0, -29, Math.toRadians(90.00)))
                .lineTo(new Vector2d(46, -55))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))  // Velocity constraint
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))  // Acceleration constraint
                .build();


        TrajectorySequence trajectory1uhoh = drive.trajectorySequenceBuilder(new Pose2d(-0.07, -33.74, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(40, -64, Math.toRadians(270.00)), Math.toRadians(-34.34))
                .build();



        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(-0.07, -33.74, Math.toRadians(270.00)))
                .lineToConstantHeading(new Vector2d(40, -64))
                .build();

        // back a little after trajectory 1
        TrajectorySequence backalittle = drive.trajectorySequenceBuilder(trajectory1.end())
                .back(15)
                .build();


        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(trajectory1uhoh.end())

                .splineToLinearHeading(new Pose2d(-5, -30, Math.toRadians(270.00)), Math.toRadians(145.81))
                .build();


        TrajectorySequence trajectory4 = drive.trajectorySequenceBuilder(backalittle.end())
                .lineToLinearHeading(new Pose2d(-3, -29, Math.toRadians(270.00)))
                .build();

        //back a little after trajectory 2

        TrajectorySequence backalittle2 = drive.trajectorySequenceBuilder(trajectory3.end())
                .back(5)
                .build();
        // -33.74




        TrajectorySequence FirstPush = drive.trajectorySequenceBuilder(trajectory0.end())
                .lineToLinearHeading(new Pose2d(33.59, -47.38, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(33.59, -4.37))
                .lineToConstantHeading(new Vector2d(40, -4.37))
                .lineTo(new Vector2d(47, -55))//y = - 64
                .lineTo(new Vector2d(37, -53))//y = - 64
                .lineTo(new Vector2d(37, -64))//y = - 64
                .build();

        //First Push alternative that poushes two
        TrajectorySequence SecondPush =  drive.trajectorySequenceBuilder(trajectory3.end())

                .lineToLinearHeading(new Pose2d(33.59, -40, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(49, -4.37))
                .lineTo(new Vector2d(49, -62))
                .build();

        TrajectorySequence shit = drive.trajectorySequenceBuilder(FirstPush.end())
                .lineTo(new Vector2d(34.63, -39.67))
                .lineTo(new Vector2d(35.81, -12.98))
                .lineTo(new Vector2d(48.27, -6.60))
                .lineTo(new Vector2d(47.38, -58.95))
                .build();




        TrajectorySequence FirstPushTest = drive.trajectorySequenceBuilder(trajectory0.end())
                .splineTo(new Vector2d(32.26, -33.74), Math.toRadians(33.98))
                .splineToLinearHeading(new Pose2d(40, -8.68, Math.toRadians(90.00)), Math.toRadians(22.74))
                .build();












        // slow down first one because inertia



//               .lineToLinearHeading(new Pose2d(37.00, -41.45, Math.toRadians(270.00)))
//               .splineTo(new Vector2d(36.41, -12), Math.toRadians(90.94))
//                .lineTo(new Vector2d(48.57, -12))
//                .lineTo(new Vector2d(47.98, -58.65))

//               .lineTo(new Vector2d(36.11, -45.31))
//               .lineTo(new Vector2d(37.45, -8.23))
//               .lineTo(new Vector2d(47.68, -4.82))
//               .lineTo(new Vector2d(48.42, -55.98))



        TrajectorySequence backalittle3 = drive.trajectorySequenceBuilder(FirstPushTest.end())
                .back(15)
                .build();



        wrist.setPosition(0);
        elevator.setTargetPosition(2000);                  //FIRST SPECIMEN
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.6);
        drive.followTrajectorySequence(trajectory0);
        elevator.setTargetPosition(1400);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.6);
        sleep(500);
        claw.setPosition(0.7);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.6);

//     claw position 1 is close
        //0.7 is open
        //RETURN TO WALL AND INTAKE
        drive.followTrajectorySequence(FirstPush);
        drive.followTrajectorySequence(shit);
        sleep(500);
        claw.setPosition(1);
        elevator.setTargetPosition(2000);   //set higher probably               //BACK TO SUBMERSIBLE
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); //2nd SPECIMEN SCORED
        elevator.setPower(0.6);
        sleep(500);
        drive.followTrajectorySequence(trajectory3);
        //drive.followTrajectorySequence(backalittle2);
        elevator.setTargetPosition(1400);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.6);
        sleep(500);
        claw.setPosition(0.7);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.6);
        sleep(500);
        drive.followTrajectorySequence(trajectory1uhoh);
        //drive.followTrajectorySequence(backalittle);
        sleep(500);
        claw.setPosition(1);
        elevator.setTargetPosition(2000);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.6);
        sleep(500);
        drive.followTrajectorySequence(trajectory3);
        //drive.followTrajectorySequence(backalittle2);
        elevator.setTargetPosition(1400);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.6);
        sleep(500);
        claw.setPosition(0.7);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.6);
        //drive.followTrajectorySequence(SecondPush);
        drive.followTrajectorySequence(park);


        /*
         elevator.setTargetPosition(2400);                 //BACK TO SUBMERSIBLE
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); //2nd SPECIMEN SCORED
        elevator.setPower(0.6);
        sleep(500);
        drive.followTrajectorySequence(trajectory3);
        drive.followTrajectorySequence(backalittle2);
        sleep(500);
        elevator.setTargetPosition(1500);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.9);
        sleep(500);
        claw.setPosition(0.7);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.9);


        drive.followTrajectorySequence(trajectory1uhoh);//park
        drive.followTrajectorySequence(backalittle);
        claw.setPosition(1);
        sleep(500);

        elevator.setTargetPosition(2400);                 //BACK TO SUBMERSIBLE
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); //2nd SPECIMEN SCORED
        elevator.setPower(0.6);
        sleep(500);
        drive.followTrajectorySequence(trajectory3);
        drive.followTrajectorySequence(backalittle2);
        sleep(500);
        elevator.setTargetPosition(1500);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.9);
        sleep(500);
        claw.setPosition(0.7);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.9);
        drive.followTrajectorySequence(FirstPush);
                     */

















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



