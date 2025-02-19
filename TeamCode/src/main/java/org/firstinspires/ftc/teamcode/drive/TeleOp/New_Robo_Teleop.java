package org.firstinspires.ftc.teamcode.drive.TeleOp;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name="New_Robo_Teleop")
//@Disabled

public class New_Robo_Teleop extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive = null;
    private DcMotor elevatorL, elevatorR, HangL, HangR = null;
    private Servo  intake_wrist, intake_claw, outtake_arm,outtake_wrist,outtake_rotation,
            outtake_claw, intake_extension,  intake_rotation = null;






    @Override
    public void runOpMode() {


        //.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
/////////////////////////////////////////////////////////////////////////////////////////////////////////
        //HwMap all items
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        elevatorL = hardwareMap.get(DcMotor.class, "elevator_L");
        elevatorR = hardwareMap.get(DcMotor.class, "elevator_R");
        elevatorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        HangL = hardwareMap.get(DcMotor.class, "HangL");
        HangR = hardwareMap.get(DcMotor.class, "HangR");


        intake_rotation = hardwareMap.get(Servo.class, "intake_rotation");
        intake_wrist = hardwareMap.get(Servo.class, "intake_wrist");
        intake_claw = hardwareMap.get(Servo.class, "intake_claw");
        intake_extension = hardwareMap.get(Servo.class, "intake_extension");


        outtake_arm = hardwareMap.get(Servo.class, "outtake_arm");
        outtake_wrist = hardwareMap.get(Servo.class, "outtake_wrist");
        outtake_rotation = hardwareMap.get(Servo.class, "outtake_rotation");
        outtake_claw = hardwareMap.get(Servo.class, "outtake_claw");


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.update();

        waitForStart();
        runtime.reset();




        while (opModeIsActive()) {
            double max;



            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            double slidePower = -gamepad2.right_stick_y;

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;


            if (gamepad2.right_bumper)
                intake_extension.setPosition(1);
            intake_wrist.setPosition(1);
            if (gamepad2.left_bumper)
                intake_extension.setPosition(0);
            intake_wrist.setPosition(0);




            if (gamepad2.b) {
                intake_claw.setPosition(1);
                outtake_claw.setPosition(1);
            }else{
                intake_claw.setPosition(0);
                outtake_claw.setPosition(0);}


            if (gamepad1.dpad_up) {
                HangL.setPower(-0.7);
                HangR.setPower(-0.7);
            } else if (gamepad1.dpad_down){
                HangL.setPower(0.7);
                HangR.setPower(0.7);
            }else{
                HangL.setPower(0);
                HangL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                HangR.setPower(0);
                HangR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}





            elevatorL.setPower(slidePower);
            elevatorR.setPower(slidePower);

            telemetry.update();




            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));



            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
                rightBackPower /= max;

            }
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        }


    }}