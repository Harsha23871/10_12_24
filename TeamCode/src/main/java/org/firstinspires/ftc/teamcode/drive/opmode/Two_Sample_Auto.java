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

@Autonomous(group = "a", name="2 Sample Test")
public class Two_Sample_Auto extends LinearOpMode {
    public DcMotor elevator, armMotor = null;
    public Servo claw, wrist, bucket , intakeClaw= null;



    public void runOpMode() {
        elevator = hardwareMap.get(DcMotor.class, "elevator_motor");
        claw = hardwareMap.get(Servo.class, "claw");
        bucket = hardwareMap.get(Servo.class, "bucket");
        wrist = hardwareMap.get(Servo.class, "wrist");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        intakeClaw = hardwareMap.get(Servo.class, "intake_claw");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



//        hwMapForAuto.elevator_Scoring_Pos();  // Moves elevator to scoring position
//        hwMapForAuto.elevator_Resting_Pos();
        //elevator = hardwareMap.get(DcMotor.class,"elevator_motor");
        bucket.setPosition(0.2);


        waitForStart();

//        elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        // Moves elevator to resting position

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36, -64, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -64.00, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-56, -59, Math.toRadians(45.00)))
                .build();

        TrajectorySequence FirstGrab = drive.trajectorySequenceBuilder(new Pose2d(-58.65, -59.00, Math.toRadians(45.00)))
                .lineToLinearHeading(new Pose2d(-51, -48, Math.toRadians(90.00)))
                .build();


// add function similar to a trajectory but for arm and wrist motors like function close()








        wrist.setPosition(0.4); //PICKING UP POSITION
        armMotor.setTargetPosition(1000);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.7);
        intakeClaw.setPosition(0.7);
      //  sleep(1000);

        elevator.setTargetPosition(3600);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.7);
        drive.followTrajectorySequence(trajectory0);
        bucket.setDirection(Servo.Direction.FORWARD);
        bucket.setPosition(1);
        sleep(1000);
        bucket.setPosition(0.2);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.7);
            // 1 is close 0.7 is open
        drive.followTrajectorySequence(FirstGrab);
        intakeClaw.setPosition(0.7);//close
        wrist.setPosition(1);//releasing position!!!!
        sleep(3000);

        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        intakeClaw.setPosition(1);//open
        sleep(7000);

















        if (isStopRequested()) return;
    }}