package org.firstinspires.ftc.teamcode.Examples.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Examples.HardwareBasebot;

/**
 * Created by Adam on 2/17/2018.
 */

@TeleOp(name="Basic Drive", group="Adam")
@Disabled
public class BasicDrive extends LinearOpMode {

    private ElapsedTime runtime  = new ElapsedTime();
    HardwareBasebot robot    = new HardwareBasebot();   // Use a Pushbot's hardware

    public final double SPEED = -0.4;
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Status", "Controller" + gamepad1.toString());
            telemetry.update();

            //Wheel Controls

            //Version 2.3 Driver Controls
            //This is a very simple teleop code example
            //I mainly use it to debug problems and test others motors
            //Set the power of the motors to the correspoding stick on the gamepad
            robot.leftMotor.setPower(gamepad1.left_stick_y*SPEED);
            robot.rightMotor.setPower(gamepad1.right_stick_y*SPEED);

            //Arm Controls




        }
    }
}