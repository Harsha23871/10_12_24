package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ModuleTrajectories {

    public static void initialize(HardwareMap hardwareMap, LinearOpMode opMode) {
        DcMotor elevator = hardwareMap.get(DcMotor.class, "elevator_motor");
        Servo claw = hardwareMap.get(Servo.class, "claw");

        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        claw.setDirection(Servo.Direction.FORWARD);
        claw.setPosition(1);

        // Wait for the game to start
        opMode.waitForStart();
    }

//ModuleTrajectories.Name





































}
