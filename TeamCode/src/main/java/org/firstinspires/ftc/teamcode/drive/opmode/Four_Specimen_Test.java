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

@Autonomous(group = "a", name="4 Specimen  ")
public class    Four_Specimen_Test extends LinearOpMode {
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


        //        hwMapForAuto.elevator_Scoring_Pos();  // Moves elevator to scoring position
//        hwMapForAuto.elevator_Resting_Pos();
        //elevator = hardwareMap.get(DcMotor.class,"elevator_motor");
        claw.setDirection(Servo.Direction.FORWARD);
        claw.setPosition(1);
//MY NAME IS HARSHA AND I AM AN IDIOT
        waitForStart();

//        elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        // Moves elevator to resting position

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(12, -64, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        // to the submersible
        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(11, -61, Math.toRadians(270.00)), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(6, -32), Math.toRadians(34.39), // -27 OG  //- -27.5 first change
                        SampleMecanumDrive.getVelocityConstraint(52.45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();



        TrajectorySequence newSpline = drive.trajectorySequenceBuilder(trajectory0.end(), Math.toRadians(270))//goes around the submersible

//                .splineToConstantHeading(new Vector2d(35, -25), Math.toRadians(0))
//                .setTangent(180)//hit metal
//                .splineToConstantHeading(new Vector2d(10, -40), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(15, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(20, -50), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(37, -45), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(28, -40), Math.toRadians(0))//worked
//                .splineToConstantHeading(new Vector2d(30, -35), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(35, -25), Math.toRadians(180))

                .lineTo(new Vector2d(35, -13))
                .lineToLinearHeading(new Pose2d(40, -8, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(48, -50), Math.toRadians(0))


                .lineToLinearHeading(new Pose2d(43, -14, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(49, -8, Math.toRadians(90)))
                .lineTo(new Vector2d(50, -8))


                .splineToConstantHeading(new Vector2d(49, -64), Math.toRadians(0))

                .build();

        TrajectorySequence scoreTrajectory = drive.trajectorySequenceBuilder(trajectory0.end(), Math.toRadians(90))//goes around the submersible

                .splineToLinearHeading(new Pose2d(0, -32, Math.toRadians(270)), Math.toRadians(0))

                .build();
        TrajectorySequence NEWPUSH = drive.trajectorySequenceBuilder(trajectory0.end(), Math.toRadians(90))//goes around the submersible

                .lineToLinearHeading(new Pose2d(33.59, -47.38, Math.toRadians(90)))
                .build();




        armMotor.setTargetPosition(100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        wrist.setPosition(0);
        elevator.setTargetPosition(1900); // 2000                 //FIRST SPECIMEN old:2000 put to all others
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); ///////////// Reduced by 1000
        elevator.setPower(0.9);
        elevator2.setTargetPosition(-1900);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setPower(-0.9);
        drive.followTrajectorySequence(trajectory0);
        elevator.setTargetPosition(1100); //old: 1400 put to all others reduced by 600
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.9);
        elevator2.setTargetPosition(-1100);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setPower(-0.9);


        sleep(200); // old 1000
        claw.setPosition(0.7);
        sleep(500);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.9);
        elevator2.setTargetPosition(0);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setPower(-0.9);


        drive.followTrajectorySequence(newSpline);


        sleep(500);
        claw.setPosition(1);
        sleep(100);

        elevator.setTargetPosition(2000);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8);
        elevator2.setTargetPosition(-2000);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setPower(-0.8);
        sleep(500);


        elevator.setTargetPosition(1100);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8);
        elevator2.setTargetPosition(-1100);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setPower(-0.8);


        sleep(250);
        claw.setPosition(0.8);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8);
        elevator2.setTargetPosition(0);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setPower(-0.8);


        sleep(250);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(500);
        claw.setPosition(1);
        sleep(250);
        elevator.setTargetPosition(2000);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8);
        elevator2.setTargetPosition(-2000);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setPower(-0.8);
        sleep(500);


        elevator.setTargetPosition(900);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8);
        elevator2.setTargetPosition(-900);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setPower(-0.8);
        sleep(750);
        claw.setPosition(0.8);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8);
        elevator2.setTargetPosition(0);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setPower(-0.8);

        //drive.followTrajectorySequence(SecondPush);


        if (isStopRequested()) return;
    }}
