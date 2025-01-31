package org.firstinspires.ftc.teamcode.drive.TeleOp;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
//@TeleOp(group = "drive")
@TeleOp(name = "TeleOpLgChamp")
public class TeleOpLgChamp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive, armMotor, elevator = null;
    private Servo claw, bucket, wrist, intakeClaw, intake_extension = null;
    private ElapsedTime armTimer = new ElapsedTime();
    double ticks = 2786.2;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        elevator = hardwareMap.get(DcMotor.class, "elevator_motor");
        claw = hardwareMap.get(Servo.class, "claw");
        bucket = hardwareMap.get(Servo.class, "bucket");
        wrist = hardwareMap.get(Servo.class, "wrist");
        intakeClaw = hardwareMap.get(Servo.class, "intake_claw");
        intake_extension = hardwareMap.get(Servo.class, "intake_extension");
        //elevatorHang = hardwareMap.get(DcMotor.class, "elevator_hang");
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

////////////////////////////////////////////////////////////////////////////////////
        //drive
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);


//        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(REVERSE);
        elevator.setDirection(FORWARD);

        waitForStart();
        while (opModeIsActive()) {
        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
//            runtime.reset();


            // run until the end of the match (driver presses STOP)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                double max;


                //game pad 1
                double slidePower = -gamepad2.right_stick_y;
//            double alpha = -gamepad2.right_stick_y;

                //drive G1
//            double elevatorUp = sigma;
//            double elevatorDown = alpha;

//            float x = gamepad2.right_stick_y;
//            if (gamepad2.right_stick_y > 0) {
//                elevator.setPower(x);
//            }
//            elevator.setPower(0);



           /* public void elevatorThing {
                elevator.setTargetPosition(elevator_scoring_pos);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(0.8);
                while (elevator.isBusy()) {}
*/



//           if (gamepad2.y) {
//
//                elevator.setTargetPosition(2000);
//                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                elevator.setPower(0.8);
//
//            }
//
//            if (gamepad2.a) {
//
//                elevator.setTargetPosition(1400);
//                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                elevator.setPower(0.8);
//                sleep(500);
//                claw.setPosition(0.7);
//                elevator.setTargetPosition(0);
//                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                elevator.setPower(0.8);
//
//            }
//            if (gamepad2.back)
//                elevator.setTargetPosition(3400);

                if (gamepad2.right_bumper)
                    intake_extension.setPosition(1);

                if (gamepad2.left_bumper)
                    intake_extension.setPosition(0);


            /*if (gamepad1.dpad_down){

            // Loop until the encoder position is 700 or greater
            while (opModeIsActive() && elevator.getCurrentPosition() < 700) {
                elevator.setTargetPosition(750);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(0.6);

//                telemetry.addData("Encoder Position", elevator.getCurrentPosition());
//                telemetry.update();

            }
//
}*/                 //UNCOMMENTTTTT MANUAL ELEVATOR
//            if (gamepad2.a) {  /* elevator down */ // might require boolean controller
//                elevator.setPower(-0.8);
//
//            }else if(gamepad2.y) { /* elevator up */
//                elevator.setPower(0.8);
//
//            }else{
//                elevator.setPower(0);}
//                elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                if (gamepad2.dpad_up) {
                    armMotor.setPower(-0.7);
                }else if (gamepad2.dpad_down)
                    armMotor.setPower(0.7);
                else{
                    armMotor.setPower(0);
                    armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}
                /*elevator.setPower();*/

//
                if (gamepad2.dpad_left)
                    wrist.setPosition(0.5);
                if (gamepad2.dpad_right)
                    wrist.setPosition(0);
//            if(gamepad1.b)
//                intakeClaw.setPosition(1);
//            if(gamepad1.x)
//                intakeClaw.setPosition(0.7);

                if (gamepad2.b) {
                    claw.setDirection(Servo.Direction.FORWARD);
                    claw.setPosition(0.7);
                    intakeClaw.setPosition(0.5);
                } else {
                    claw.setPosition(1);
                    intakeClaw.setPosition(1);
                }
                //bucket
                if (gamepad2.x) {
                    bucket.setDirection(Servo.Direction.FORWARD);
                    bucket.setPosition(1);
                } else {
                    bucket.setPosition(0.2);
                }


                elevator.setPower(slidePower);




            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
            runtime.reset();
        }
    }
}}
