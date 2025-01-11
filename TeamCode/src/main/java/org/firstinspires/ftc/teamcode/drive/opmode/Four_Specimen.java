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

import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.CMAESOptimizer;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "a", name="4 Specimen Test ")
public class Four_Specimen extends LinearOpMode {
    public DcMotor elevator,armMotor = null;
    public Servo claw,wrist,intake_extension,intakeClaw = null;



    public void runOpMode() {
        elevator = hardwareMap.get(DcMotor.class, "elevator_motor");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        intakeClaw = hardwareMap.get(Servo.class, "intake_claw");
        intake_extension = hardwareMap.get(Servo.class, "intake_extension");
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

      //  static void myMethod() {
      //      // code to be executed
     //   }







        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(11, -61, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        // to the submersible
        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(11, -61, Math.toRadians(270.00)))
                .lineTo(new Vector2d(0, -31))




                .build();


        TrajectorySequence trajectory1uhoh = drive.trajectorySequenceBuilder(new Pose2d(-0.07, -33.74, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(40, -64, Math.toRadians(95)), Math.toRadians(-34.34))
                .build();


        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(-0.07, -33.74, Math.toRadians(270.00)))
                .lineToConstantHeading(new Vector2d(40, -64))
                .build();


        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(trajectory1uhoh.end())

                .splineToLinearHeading(new Pose2d(-5, -30, Math.toRadians(270.00)), Math.toRadians(145.81))
                .build();


        TrajectorySequence FirstPush = drive.trajectorySequenceBuilder(trajectory0.end())
                .lineToLinearHeading(new Pose2d(33.59, -47.38, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(33.59, -4.37))
                .lineToConstantHeading(new Vector2d(40, -4.37))
                .lineTo(new Vector2d(47, -55))//y = - 64
                .lineTo(new Vector2d(37, -53))//y = - 64
                .lineTo(new Vector2d(37, -64))//y = - 64
                .build();


//    TrajectorySequence Spline = drive.trajectorySequenceBuilder(new Pose2d(0.00, -31.00, Math.toRadians(90.00)))
//   .lineTo(new Vector2d(28.00, -42.00))
//   .splineTo(new Vector2d(48, -11.79), Math.toRadians(5.00))
//    .lineTo(new Vector2d(48, -62.36))
//     .build();




    //    TrajectorySequence SigmaSigmaBoi = drive.trajectorySequenceBuilder(trajectory0.end())
       //         .lineToLinearHeading(new Pose2d(35.5, -41, Math.toRadians(55)))
        //        .build();

     //   TrajectorySequence dropoff = drive.trajectorySequenceBuilder(SigmaSigmaBoi.end())
      //          .lineToLinearHeading(new Pose2d(42.34, -45, Math.toRadians(-45)))
        //        .build();


        wrist.setPosition(0.4);
        elevator.setTargetPosition(2000);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.6);
        drive.followTrajectorySequence(trajectory0);
        elevator.setTargetPosition(1400);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.6);


        //        armMotor.setTargetPosition(1100);
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armMotor.setPower(0.7);
//        intakeClaw.setPosition(0.6);
        sleep(500);

        claw.setPosition(0.7);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.6);

//     claw position 1 is close
        //0.7 is open
        //RETURN TO WALL AND INTAKE
  //      drive.followTrajectorySequence(Spline);
//        drive.followTrajectorySequence(FirstPush);
   //     drive.followTrajectorySequence(SigmaSigmaBoi);

      //  armMotor.setTargetPosition(1300); // Pickup
      //  armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     //   armMotor.setPower(0.8);
   //     sleep(500);//1000
   //     intakeClaw.setPosition(1);


      //  elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(1000);

        armMotor.setTargetPosition(1100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        sleep(1000);
    //    drive.followTrajectorySequence(dropoff);

        intakeClaw.setPosition(1);
    //    elevator.setTargetPosition(2000);   //set higher probably               //BACK TO SUBMERSIBLE
    //    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); //2nd SPECIMEN SCORED
     //   elevator.setPower(0.6);
        sleep(1000);
   //     drive.followTrajectorySequence(trajectory3);

//        elevator.setTargetPosition(1400);
//        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        elevator.setPower(0.6);
        sleep(500);
        intakeClaw.setPosition(0.7);
//        elevator.setTargetPosition(0);
//        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        elevator.setPower(0.6);
        sleep(500);
     //   drive.followTrajectorySequence(trajectory1uhoh);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(500);
        claw.setPosition(1);
        elevator.setTargetPosition(2000);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.6);
        sleep(500);
        drive.followTrajectorySequence(trajectory3);

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

        claw.setPosition(1);
        sleep(500);

        elevator.setTargetPosition(2400);                 //BACK TO SUBMERSIBLE
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); //2nd SPECIMEN SCORED
        elevator.setPower(0.6);
        sleep(500);
        drive.followTrajectorySequence(trajectory3);

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


    }}