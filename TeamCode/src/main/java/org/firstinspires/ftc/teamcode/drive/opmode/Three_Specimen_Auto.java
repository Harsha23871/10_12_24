package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "a", name="3 Specimen Auto ")
public class    Three_Specimen_Auto extends LinearOpMode {
    public DcMotor elevator, armMotor = null;
    public Servo claw,wrist = null;

//    public void ElevatorScoreUp() { // Need to add Elevator score down they cant be in the same function as score because traj in middle
//        wrist.setPosition(0);
//        elevator.setTargetPosition(2000);                  //FIRST SPECIMEN old:2000 put to all others
//        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); ///////////// Reduced by 1000
//        elevator.setPower(0.6);
//
//    }

    public void runOpMode() {
        elevator = hardwareMap.get(DcMotor.class,"elevator_motor");
        claw =  hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);



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
        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(11, -61, Math.toRadians(270.00)), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(4, -28.75), Math.toRadians(34.39), // -27 OG  //- -27.5 first change
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();


        TrajectorySequence trajectory1uhoh = drive.trajectorySequenceBuilder(new Pose2d(-0.07, -33.74, Math.toRadians(270)), Math.toRadians(270))
                .lineToLinearHeading(new Pose2d(40, -64, Math.toRadians(95)),

                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        /*TrajectorySequence newSpline = drive.trajectorySequenceBuilder(trajectory0.end(), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(36, -42), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(42.34, -4.82))
                .lineToConstantHeading(new Vector2d(49.16, -59.39))

                .build(); */

        TrajectorySequence trajectory1uhohtest = drive.trajectorySequenceBuilder(new Pose2d(0.00, -33.00, Math.toRadians(270.00)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(40.00, -64.00, Math.toRadians(95.00)), Math.toRadians(0.00))
                .build();



        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(-0.07, -33.74, Math.toRadians(270.00)), Math.toRadians(270))
                .lineToConstantHeading(new Vector2d(40, -64))
                .build();



        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(trajectory1uhoh.end(), Math.toRadians(270))

                .lineToLinearHeading(new Pose2d(-7, -29, Math.toRadians(-90))) // 270 (-5) try normal 90

                //
                .build();

        TrajectorySequence trajectory4 = drive.trajectorySequenceBuilder(trajectory1uhoh.end(), Math.toRadians(270))
                .lineToLinearHeading(new Pose2d(-9, -29, Math.toRadians(-90)))
                .back(5)

                .build();




        TrajectorySequence FirstPush = drive.trajectorySequenceBuilder(trajectory0.end(), Math.toRadians(270))
                .lineToLinearHeading(new Pose2d(33.59, -47.38, Math.toRadians(90)))//weird turny
                .lineToConstantHeading(new Vector2d(33.59, -4.37)) //straight
                .lineToConstantHeading(new Vector2d(43, -4.37))//right
                .lineTo(new Vector2d(47, -55))//y = - 64 //backkkk
                .lineTo(new Vector2d(35, -51))//y = - 64 left
                .lineTo(new Vector2d(35, -64))//y = - 64 towards human
                .build();









        armMotor.setTargetPosition(100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        wrist.setPosition(0);
        elevator.setTargetPosition(1900); // 2000                 //FIRST SPECIMEN old:2000 put to all others
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); ///////////// Reduced by 1000
        elevator.setPower(0.9);
        drive.followTrajectorySequence(trajectory0);
        elevator.setTargetPosition(1100); //old: 1400 put to all others reduced by 600
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.9);
        sleep(200); // old 1000
        claw.setPosition(0.7);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.9);
//
//     claw position 1 is close
        //0.7 is open
        //RETURN TO WALL AND INTAKE





//        drive.followTrajectorySequence(newSpline);
        drive.followTrajectorySequence(FirstPush);

        sleep(500); // added sleep recent change // may change

        // drive.followTrajectorySequence(SecondFirstPush );
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);
        claw.setPosition(1);
        sleep(100);
        elevator.setTargetPosition(2000);                 //BACK TO SUBMERSIBLE
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); //2nd SPECIMEN SCORED
        elevator.setPower(0.8);

        sleep(500);

        drive.followTrajectorySequence(trajectory3);
        //drive.followTrajectorySequence(backalittle2);
        elevator.setTargetPosition(900);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8);
        sleep(250);
        claw.setPosition(0.8);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8);
        sleep(250);
        drive.followTrajectorySequence(trajectory1uhohtest);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //drive.followTrajectorySequence(backalittle);
        sleep(500);
        claw.setPosition(1);
        sleep(250);
        elevator.setTargetPosition(2000);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8);
        sleep(500);
        drive.followTrajectorySequence(trajectory4);
        //drive.followTrajectorySequence(backalittle2);
        elevator.setTargetPosition(900);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8);
        sleep(750);
        claw.setPosition(0.8);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8);
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
//sourish likes men and hawktuah from andrew 2/20/25