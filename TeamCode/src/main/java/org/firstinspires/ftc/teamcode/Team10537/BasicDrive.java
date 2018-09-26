package org.firstinspires.ftc.teamcode.Team10537;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Adam on 2/17/2018.
 */

@TeleOp(name="Basic Drive", group="Adam")
@Disabled
public class BasicDrive extends LinearOpMode {

    private ElapsedTime runtime  = new ElapsedTime();
    MecanumBasebot robot    = new MecanumBasebot();   // Use a Pushbot's hardware

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

            robot.leftFrontMotor.setPower(-gamepad1.left_stick_y*SPEED);
            robot.leftRearMotor.setPower(-gamepad1.left_stick_y*SPEED);
            robot.rightFrontMotor.setPower(gamepad1.right_stick_y*SPEED);
            robot.rightRearMotor.setPower(gamepad1.right_stick_y*SPEED);
            robot.lift.setPower((gamepad2.left_stick_y));

            //Arm Controls
            while(gamepad1.x){
                robot.leftHand.setPosition(1);
                robot.rightHand.setPosition(1);
            }
            while(gamepad1.y){
                robot.leftHand.setPosition(0);
                robot.rightHand.setPosition(0);
            }



        }
    }
}