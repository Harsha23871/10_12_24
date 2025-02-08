package org.firstinspires.ftc.teamcode.drive.TeleOp; //opmode

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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "a", name="h")
public class h extends LinearOpMode {
    public DcMotor elevator, armMotor = null;
    public Servo claw, wrist, bucket , intakeClaw, intake_extension= null;



    public void runOpMode() {
        elevator = hardwareMap.get(DcMotor.class, "elevator_motor");
        claw = hardwareMap.get(Servo.class, "claw");
        bucket = hardwareMap.get(Servo.class, "bucket");
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
        bucket.setPosition(0.2);


        waitForStart();
        wrist.setPosition(0);

//        elevator.setMode(DcMotorEx.RunMode.ST
//
//        +OP_AND_RESET_ENCODER);
//        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        // Moves elevator to resting position

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(47.38, -33.44, Math.toRadians(90));
        drive.setPoseEstimate(startPose);



        TrajectorySequence trajectorysigma = drive.trajectorySequenceBuilder(new Pose2d(47.38, -33.44, Math.toRadians(90.00)), Math.toRadians(270))
                .lineTo(new Vector2d(60, -33.44))
                .build();

        //-57   //-59






//
//


// add function similar to a trajectory but for arm and wrist motors like function close()







        intake_extension.setPosition(0);
        sleep(500);
        intakeClaw.setPosition(0.5); //open
        sleep(500);
        wrist.setPosition(0);
        sleep(500);
        armMotor.setTargetPosition(1500);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        sleep(1000);
        intakeClaw.setPosition(1);
        sleep(1000);
        wrist.setPosition(0.5);
        sleep(1000);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        sleep(1000);
        intakeClaw.setPosition(0.5);
        sleep(500);
        bucket.setDirection(Servo.Direction.FORWARD);

        sleep(500);

        wrist.setPosition(0);
        sleep(500);
        intake_extension.setPosition(0);
        sleep(500);
        intakeClaw.setPosition(0.5);
        sleep(500);
        armMotor.setTargetPosition(1500);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        bucket.setPosition(1);
        sleep(500);
        bucket.setPosition(0.2);
        drive.followTrajectorySequence(trajectorysigma);

        sleep(500); // 100


        intakeClaw.setPosition(0.5);
        wrist.setPosition(0);
        sleep(500);
        intakeClaw.setPosition(1);
        sleep(1000);
        wrist.setPosition(0.5);
        sleep(1000);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        sleep(500);
        bucket.setDirection(Servo.Direction.FORWARD);
        intakeClaw.setPosition(0.5);
        armMotor.setTargetPosition(1100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        sleep(500);
        bucket.setPosition(1);
        sleep(500);
        bucket.setPosition(0.2);
        sleep(500); // 100






        if (isStopRequested()) return;
    }}