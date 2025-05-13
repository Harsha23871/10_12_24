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

@Autonomous(group = "a", name="quesoBowlSpecimen  ")
public class    quesoBowlSpecimen extends LinearOpMode {
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
        Pose2d startPose = new Pose2d(48, -64, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        TrajectorySequence Preload = drive.trajectorySequenceBuilder(new Pose2d(48, -64), Math.toRadians(270))

                .lineTo(new Vector2d(0, -32))

                .build();

        TrajectorySequence Pickup = drive.trajectorySequenceBuilder(Preload.end(), Math.toRadians(270))
                .lineToLinearHeading(new Pose2d(0.00, -64.00, Math.toRadians(90)))
                .build();

        TrajectorySequence Score = drive.trajectorySequenceBuilder(Pickup.end(), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(0, -32, Math.toRadians(270)))
                .build();

        drive.followTrajectorySequence(Preload);
        sleep(1000);

        drive.followTrajectorySequence(Pickup);
        sleep(1000);

        drive.followTrajectorySequence(Score);
        sleep(1000);

        drive.followTrajectorySequence(Pickup);
        sleep(1000);

        drive.followTrajectorySequence(Score);
        sleep(1000);

        drive.followTrajectorySequence(Pickup);
        sleep(1000);

        drive.followTrajectorySequence(Score);
        sleep(1000);



        if (isStopRequested()) return;
    }}
