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

@Autonomous(group = "a", name="2_specimen_score")
public class Two_specimenAuto extends LinearOpMode {
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
// to the observation zone
        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(trajectory0.end())
               .splineToConstantHeading(new Vector2d(48, -58.5), Math.toRadians(34.39),


        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)


                )

                .build();
        //back to submersible
        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(trajectory1.end())
                .splineToConstantHeading(new Vector2d(0, -40), Math.toRadians(34.39),//(0,-40) is a test coordinate

                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)


                )
                .build();



        elevator.setTargetPosition(1900);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.6);

        drive.followTrajectorySequence(trajectory0);
        elevator.setTargetPosition(1500);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.9);

        sleep(3000);
         //0.7 is open
        // 1 is close
        claw.setPosition(0.7);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.9);

        sleep(1000);
        drive.followTrajectorySequence(trajectory1);
        drive.turn(Math.toRadians(185));

        claw.setPosition(1);

        elevator.setTargetPosition(1900);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.6);
        drive.followTrajectorySequence(trajectory2);

        elevator.setTargetPosition(1500);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.9);

        sleep(3000);

        claw.setPosition(0.7);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.9);

        sleep(1000);
        drive.followTrajectorySequence(trajectory1);















// 35.81,-37.15 start pos
        if (isStopRequested()) return;













    }   }



