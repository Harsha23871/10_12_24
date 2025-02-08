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

@Autonomous(group = "a", name="fourThing")
public class fourThing extends LinearOpMode {
    public DcMotor elevator, armMotor = null;
    public Servo claw, wrist, bucket , intakeClaw, intake_extension= null;


    //
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


        TrajectorySequence FirstHang =  drive.trajectorySequenceBuilder(new Pose2d(11, -61, Math.toRadians(270.00)), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-8, -29), Math.toRadians(34.39), // -27 OG  //- -27.5 first change
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        //to the parking zone
        TrajectorySequence threeOnRight = drive.trajectorySequenceBuilder(new Pose2d(-8, -29), Math.toRadians(270))
                .splineTo(new Vector2d(49.37, -40.16), Math.toRadians(90.00))
                .build();



        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(49.37, -40.16), Math.toRadians(270))
                .strafeRight(40)
                .build();



        //////////////                                                                                      // -33.74
// Encoder + Trajectory4/3
        /////////////////
//

















        // slow down first one because inertia



//               .lineToLinearHeading(new Pose2d(37.00, -41.45, Math.toRadians(270.00)))
//               .splineTo(new Vector2d(36.41, -12), Math.toRadians(90.94))
//                .lineTo(new Vector2d(48.57, -12))
//                .lineTo(new Vector2d(47.98, -58.65))

//               .lineTo(new Vector2d(36.11, -45.31))
//               .lineTo(new Vector2d(37.45, -8.23))
//               .lineTo(new Vector2d(47.68, -4.82))
//               .lineTo(new Vector2d(48.42, -55.98))



/////////////////////////////////////////////////////////////////////////////////////////////////
        wrist.setPosition(0);
        elevator.setTargetPosition(1900); // 2000                 //FIRST SPECIMEN old:2000 put to all others
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); ///////////// Reduced by 1000
        elevator.setPower(0.8);
        drive.followTrajectorySequence(FirstHang);
        elevator.setTargetPosition(950); //old: 1400 put to all others reduced by 600
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8);
        sleep(200); // old 1000
        claw.setPosition(0.7);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8);

//     claw position 1 is close
        //0.7 is
        intakeClaw.setPosition(0.7);
        armMotor.setTargetPosition(1100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        drive.followTrajectorySequence(threeOnRight);

        sleep(500); // added sleep recent change // may change

        armMotor.setTargetPosition(1500);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        sleep(500);//1000
        intakeClaw.setPosition(1);
        sleep(500);
        wrist.setPosition(0.5);//releasing position!!!!
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        sleep(500);
        intakeClaw.setPosition(0.5);
        sleep(500);
        bucket.setPosition(1);
        sleep(200);
        bucket.setPosition(0.5);




        drive.followTrajectorySequence(right);



        armMotor.setTargetPosition(1500);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        sleep(500);//1000
        intakeClaw.setPosition(1);
        sleep(500);
        wrist.setPosition(0.5);//releasing position!!!!
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        sleep(500);
        intakeClaw.setPosition(0.5);
        sleep(500);
        bucket.setPosition(1);
        sleep(200);
        bucket.setPosition(0.5);


































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