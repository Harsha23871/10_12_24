package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "a", name="Queso Sample Mi+rror")
public class Quesosamplemirror extends LinearOpMode {
    public DcMotor elevator, armMotor, elevator2 = null;
    public Servo claw, wrist, bucket , intakeClaw, intake_extension= null;



    public void runOpMode() {
        elevator = hardwareMap.get(DcMotor.class, "elevator_motor");
        elevator2 = hardwareMap.get(DcMotor.class, "elevator_motor2");
        claw = hardwareMap.get(Servo.class, "claw");
        bucket = hardwareMap.get(Servo.class, "bucket");
        wrist = hardwareMap.get(Servo.class, "wrist");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        intakeClaw = hardwareMap.get(Servo.class, "intake_claw");
        intake_extension = hardwareMap.get(Servo.class, "intake_extension");
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);


//        hwMapForAuto.elevator_Scoring_Pos();  // Moves elevator to scoring position
//        hwMapForAuto.elevator_Resting_Pos();
        //elevator = hardwareMap.get(DcMotor.class,"elevator_motor");
        bucket.setPosition(0.9);


        waitForStart();


//        elevator.setMode(DcMotorEx.RunMode.ST
//
//        +OP_AND_RESET_ENCODER);
//        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        // Moves elevator to resting position

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36, -64, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -64.00, Math.toRadians(90.00)), Math.toRadians(270))
                .lineToLinearHeading(new Pose2d(-57, -59, Math.toRadians(45.00)))
                .build();
        //-57   //-59





        TrajectorySequence FirstGrab = drive.trajectorySequenceBuilder(trajectory0.end(), Math.toRadians(270))
                .lineToLinearHeading(new Pose2d(-51.5, -45.5, Math.toRadians(90.00))) //-45
                .build();

        TrajectorySequence FirstScore = drive.trajectorySequenceBuilder(FirstGrab.end(), Math.toRadians(270))
                .lineToLinearHeading(new Pose2d(-58, -60, Math.toRadians(45)))
                .build();                         //-57   //-59

        TrajectorySequence SecondGrab = drive.trajectorySequenceBuilder(FirstScore.end(), Math.toRadians(270))
                .lineToLinearHeading(new Pose2d(-62, -45.5, Math.toRadians(90.00))) //-45.5
                .build();

        TrajectorySequence SecondScore = drive.trajectorySequenceBuilder(SecondGrab.end(), Math.toRadians(270))
                .lineToLinearHeading(new Pose2d(-58, -60, Math.toRadians(45)))
                .build();                         //-57   //-59

        TrajectorySequence ThirdGrab = drive.trajectorySequenceBuilder(SecondScore.end(), Math.toRadians(270))
                .lineToLinearHeading(new Pose2d(-61, -44, Math.toRadians(120))) //-46
                .build();

        TrajectorySequence ThirdScore = drive.trajectorySequenceBuilder(ThirdGrab.end(), Math.toRadians(270))
                .lineToLinearHeading(new Pose2d(-58, -60, Math.toRadians(45)))
                .build();                         //-57   //-59



        TrajectorySequence extraSample = drive.trajectorySequenceBuilder(ThirdScore.end(), Math.toRadians(270))
                .lineToLinearHeading(new Pose2d(-23.5, -58, Math.toRadians(0)))
                .build();

        TrajectorySequence extraSampleScore = drive.trajectorySequenceBuilder(extraSample.end(), Math.toRadians(270))
                .lineToLinearHeading(new Pose2d(-58, -60, Math.toRadians(45)))
                .build();

     /*   TrajectorySequence ParkieParkie = drive.trajectorySequenceBuilder(new Pose2d(-57.00, -59.00, Math.toRadians(90.00)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(180.00)), Math.toRadians(10.00))
                .build();*/



//
//


// add function similar to a trajectory but for arm and wrist motors like function close()







        intake_extension.setPosition(1);

        wrist.setPosition(0.6); // was 0.4

        armMotor.setTargetPosition(800);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        intakeClaw.setPosition(0.75);
//        intakeClaw.setPosition(0.75);
        //  sleep(1000);

        elevator.setTargetPosition(3500);elevator2.setTargetPosition(-3500);//3400 elevator used to be
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8);elevator2.setPower(-0.8);

        drive.followTrajectorySequence(trajectory0);
        wrist.setPosition(0.6); //0.9 was 0.37 from teleop change to 1
        ///////////////
        sleep(100); // 1000
        ////////////

        bucket.setDirection(Servo.Direction.FORWARD);
        bucket.setPosition(-0.9); // 0.9


        //////////////////////
        sleep(900); //1000
        /////////////////////
//        intakeClaw.setPosition(0.75);THISONE
        bucket.setPosition(0.9);
        elevator.setTargetPosition(0); elevator2.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8); elevator2.setPower(-0.8);
        drive.followTrajectorySequence(FirstGrab);

        // 1 is close 0.7 is open

//



        //////////////////////////////
        //  sleep(1000); // 1000
        /////////////////////////////

        armMotor.setTargetPosition(1250); //1500
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        sleep(500);//1000
        intakeClaw.setPosition(1);
        sleep(500);
        wrist.setPosition(1);//0//releasing position!!!! not 0.37
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        sleep(600);

        //////////////////////////////
//        sleep(1000); // 1000
        //////////////////////////////

//        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeClaw.setPosition(0.75);
        sleep(500);
        wrist.setPosition(0.6); //
        armMotor.setTargetPosition(800); // 1100
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);


        elevator.setTargetPosition(3500); elevator2.setTargetPosition(-3500);//used to be 3400
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8);  elevator2.setPower(-0.8);


        elevator.setTargetPosition(3400);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8);
        drive.followTrajectorySequence(FirstScore);

        ////////////////////////////////
        sleep(100); // 1000
        ///////////////////////////////////

        bucket.setDirection(Servo.Direction.FORWARD);
        bucket.setPosition(0);
        sleep(1000); //1000
        bucket.setPosition(0.9);
        sleep(200);

        elevator.setTargetPosition(0); elevator2.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8); elevator2.setPower(-0.8);


        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8);

        drive.followTrajectorySequence(SecondGrab);
        //sleep(1000);
        armMotor.setTargetPosition(1250);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        sleep(500); //1000
        intakeClaw.setPosition(1);
        sleep(500);
        wrist.setPosition(1);//releasing position!!!!
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
//        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(1100);
        intakeClaw.setPosition(0.75);
        sleep(500);
        wrist.setPosition(0.6); // 0 for the 0.9s

        armMotor.setTargetPosition(800);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);

        elevator.setTargetPosition(3500); elevator2.setTargetPosition(-3500);// used to be 3400
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8);  elevator2.setPower(-0.8);


        elevator.setTargetPosition(3400);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8);
        drive.followTrajectorySequence(SecondScore);

        sleep(100); //1000
        bucket.setDirection(Servo.Direction.FORWARD); /////
        bucket.setPosition(0);
        sleep(1000); //1000
        bucket.setPosition(0.9);
        sleep(200);

        elevator.setTargetPosition(0); elevator2.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8); elevator2.setPower(-0.8);


        drive.followTrajectorySequence(ThirdGrab);
        //  sleep(1000);//1000
        armMotor.setTargetPosition(1250);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        sleep(1000);
        intakeClaw.setPosition(1);
        sleep(500); //500 - 475
        wrist.setPosition(1);//releasing position!!!!
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
//        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(1000);
        intakeClaw.setPosition(0.75);
        sleep(500
        ); // 500 -475
        wrist.setPosition(0.6);
        armMotor.setTargetPosition(800);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);

        elevator.setTargetPosition(3500); elevator2.setTargetPosition(-3500);//used to be 3400
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8);  elevator2.setPower(-0.8);


        elevator.setTargetPosition(3400);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8);
        drive.followTrajectorySequence(ThirdScore);
        sleep(100);//1000

        bucket.setDirection(Servo.Direction.FORWARD);
        bucket.setPosition(0);
        sleep(1000); //1000
        bucket.setPosition(0.9);
        sleep(500);



        wrist.setPosition(0.6); // was 0.4


        armMotor.setTargetPosition(800);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);


        drive.followTrajectorySequence(extraSample);

        elevator.setTargetPosition(0); elevator2.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8); elevator2.setPower(-0.8);


        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8);

        armMotor.setTargetPosition(1250);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        sleep(100);
        intakeClaw.setPosition(1);


        sleep(500); //500 - 475
        wrist.setPosition(1);//releasing position!!!!
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
//        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(1000);
        intakeClaw.setPosition(0.75);
        sleep(500
        ); // 500 -475
        wrist.setPosition(0.6);
        armMotor.setTargetPosition(800);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        sleep(100);

        elevator.setTargetPosition(3500); elevator2.setTargetPosition(-3500);//used to be 3400
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8);  elevator2.setPower(-0.8);


        elevator.setTargetPosition(3400);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8);

        drive.followTrajectorySequence(extraSampleScore);
        //   bucket.setPosition(0);
        bucket.setPosition(-0.9);

        sleep(1000);






















        if (isStopRequested()) return;
    }}