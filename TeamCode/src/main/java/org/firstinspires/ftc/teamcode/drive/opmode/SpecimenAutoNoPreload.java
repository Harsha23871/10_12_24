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

@Autonomous(group = "b", name="NoPreloadSpecimen4  ")
public class    SpecimenAutoNoPreload extends LinearOpMode {
    public DcMotor elevator, armMotor, elevator2 = null;
    public Servo claw,wrist = null;

    public void runOpMode() {
        elevator = hardwareMap.get(DcMotor.class, "elevator_motor");
        elevator2 = hardwareMap.get(DcMotor.class, "elevator_motor2");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);




        claw.setPosition(0.7);
        waitForStart();

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(12, -64, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence Push = drive.trajectorySequenceBuilder(startPose, Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(36.00, -40.00), Math.toRadians(80.00))
                .splineToConstantHeading(new Vector2d(37.00, -18.00), Math.toRadians(100.00))
                .splineToConstantHeading(new Vector2d(44.00, -5), Math.toRadians(300.00))
                .splineToConstantHeading(new Vector2d(44.00, -52.00), Math.toRadians(270.00))

                .waitSeconds(0.01)
                .splineToConstantHeading(new Vector2d(49.00, -8), Math.toRadians(10))
                .splineToConstantHeading(new Vector2d(49.00, -52), Math.toRadians(270.00))
                .waitSeconds(0.01)
                .splineToConstantHeading(new Vector2d(36, -64.00), Math.toRadians(270.00))

                .build();
//score first
        TrajectorySequence ScorePath = drive.trajectorySequenceBuilder(new Pose2d(36, -64), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(4, -30.5, Math.toRadians(-90)), Math.toRadians(90))
                .build();
        //pick second
        TrajectorySequence PickUpPath = drive.trajectorySequenceBuilder(new Pose2d(4, -31.5), Math.toRadians(270))
//                .splineToLinearHeading(new Pose2d(36, -64, Math.toRadians(90.00)), Math.toRadians(250)) // -64
//                .waitSeconds(0.1)
                .splineToLinearHeading(new Pose2d(36.00, -55.00, Math.toRadians(90.00)), Math.toRadians(270.00))
                .splineToConstantHeading(new Vector2d(36.00, -64.00), Math.toRadians(90.00))


                .build();


//score second
        TrajectorySequence ScorePath2 = drive.trajectorySequenceBuilder(new Pose2d(36, -64), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(7, -28.5, Math.toRadians(270.00)), Math.toRadians(90)) //-30.5 y
                .build();
//ppick third
        TrajectorySequence PickUpPath2 = drive.trajectorySequenceBuilder(ScorePath2.end(), Math.toRadians(270))
                //                .splineToLinearHeading(new Pose2d(36, -64, Math.toRadians(90.00)), Math.toRadians(250)) // -64
//                .waitSeconds(0.1)
                .splineToLinearHeading(new Pose2d(36.00, -55.00, Math.toRadians(90.00)), Math.toRadians(270.00))
                .splineToConstantHeading(new Vector2d(36.00, -64.00), Math.toRadians(90.00))
                .build();
//score third

        TrajectorySequence ScorePath3 = drive.trajectorySequenceBuilder(new Pose2d(36, -64), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(10, -27.5, Math.toRadians(270.00)), Math.toRadians(90))
                .build();
//pick fouth
        TrajectorySequence PickUpPath3 = drive.trajectorySequenceBuilder(ScorePath3.end(), Math.toRadians(270))
                //                .splineToLinearHeading(new Pose2d(36, -64, Math.toRadians(90.00)), Math.toRadians(250)) // -64
//                .waitSeconds(0.1)
                .splineToLinearHeading(new Pose2d(36.00, -55.00, Math.toRadians(90.00)), Math.toRadians(270.00))
                .splineToConstantHeading(new Vector2d(36.00, -64.00), Math.toRadians(90.00))
                .build();
//score fourth
        TrajectorySequence ScorePath4 = drive.trajectorySequenceBuilder(new Pose2d(36, -64), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(13, -26.5, Math.toRadians(270.00)), Math.toRadians(90))
                .build();
// machine leaening camera code techniques below
        //limelight
        //opencv sigma
        //colordetection
        TrajectorySequence park = drive.trajectorySequenceBuilder(ScorePath.end(), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(40.00, -64.00, Math.toRadians(270)), Math.toRadians(260.00))
                .build();





        // maybe elevator
        armMotor.setTargetPosition(10); // 2000                 //FIRST SPECIMEN old:2000 put to all others
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.01); //0.01
        wrist.setPosition(0);
        drive.followTrajectorySequence(Push);
        armMotor.setTargetPosition(100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        claw.setPosition(1);//close
        sleep(80);
        elevator.setTargetPosition(1900); // 2000                 //FIRST SPECIMEN old:2000 put to all others
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); ///////////// Reduced by 1000
        elevator.setPower(0.9);
        elevator2.setTargetPosition(-1900);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setPower(-0.9);

        drive.followTrajectorySequence(ScorePath);
        elevator.setTargetPosition(1400); // 2000                 //FIRST SPECIMEN old:2000 put to all others
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); ///////////// Reduced by 1000
        elevator.setPower(0.9);
        elevator2.setTargetPosition(-1400);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setPower(-0.9);
        sleep(30); // old 1000
        claw.setPosition(0.7);

        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.9);
        elevator2.setTargetPosition(0);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setPower(-0.9);

        drive.followTrajectorySequence(PickUpPath);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//SECOND SCORE

        claw.setPosition(1);//close
        sleep(100);
        elevator.setTargetPosition(1900); // 2000                 //FIRST SPECIMEN old:2000 put to all others
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); ///////////// Reduced by 1000
        elevator.setPower(0.9);
        elevator2.setTargetPosition(-1900);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setPower(-0.9);

        drive.followTrajectorySequence(ScorePath2);
        elevator.setTargetPosition(1400); // 2000                 //FIRST SPECIMEN old:2000 put to all others
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); ///////////// Reduced by 1000
        elevator.setPower(0.9);
        elevator2.setTargetPosition(-1400);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setPower(-0.9);
        sleep(100);  // old 1000
        claw.setPosition(0.7);

        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.9);
        elevator2.setTargetPosition(0);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setPower(-0.9);

        drive.followTrajectorySequence(PickUpPath2);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //THIRD SCORE


        claw.setPosition(1);//close
        sleep(100);
        elevator.setTargetPosition(1900); // 2000                 //FIRST SPECIMEN old:2000 put to all others
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); ///////////// Reduced by 1000
        elevator.setPower(0.9);
        elevator2.setTargetPosition(-1900);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setPower(-0.9);

        drive.followTrajectorySequence(ScorePath3);
        elevator.setTargetPosition(1400); // 2000                 //FIRST SPECIMEN old:2000 put to all others
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); ///////////// Reduced by 1000
        elevator.setPower(0.9);
        elevator2.setTargetPosition(-1400);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setPower(-0.9);
        sleep(100); // old 1000
        claw.setPosition(0.7);

        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.9);
        elevator2.setTargetPosition(0);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setPower(-0.9);

        drive.followTrajectorySequence(PickUpPath3);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //FOURTH SCORE
        claw.setPosition(1);//close
        sleep(100);
        elevator.setTargetPosition(1900); // 2000                 //FIRST SPECIMEN old:2000 put to all others
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); ///////////// Reduced by 1000
        elevator.setPower(0.9);
        elevator2.setTargetPosition(-1900);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setPower(-0.9);

        drive.followTrajectorySequence(ScorePath4);
        elevator.setTargetPosition(1400); // 2000                 //FIRST SPECIMEN old:2000 put to all others
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); ///////////// Reduced by 1000
        elevator.setPower(0.9);
        elevator2.setTargetPosition(-1400);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setPower(-0.9);
        sleep(100);  // old 1000
        claw.setPosition(0.7);

        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.9);
        elevator2.setTargetPosition(0);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setPower(-0.9);

        drive.followTrajectorySequence(park);

        if (isStopRequested()) return;
    }}
