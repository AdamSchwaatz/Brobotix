package org.firstinspires.ftc.teamcode.Team10537;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Adam on 2/17/2018.
 */

@TeleOp(name="Mecanum Drive", group="Adam")
@Disabled
public class MecanumDrive extends LinearOpMode {

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

            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            robot.leftFrontMotor.setPower(v1);
            robot.rightFrontMotor.setPower(v2);
            robot.leftRearMotor.setPower(v3);
            robot.rightRearMotor.setPower(v4);

            //Arm Controls
//            if(gamepad1.dpad_up){
//                robot.leftArm.setPower(0.4);
//                robot.rightArm.setPower(0.4);
//            }else if (gamepad1.dpad_down) {
//                robot.leftArm.setPower(-0.4);
//                robot.rightArm.setPower(-0.4);
//            }else {
//                robot.leftArm.setPower(0);
//                robot.rightArm.setPower(0);
//            }
            while(gamepad1.x){
                robot.rightHand.setPosition(1);
                robot.leftHand.setPosition(1);
            }
            while(gamepad1.y){
                robot.rightHand.setPosition(0);
                robot.leftHand.setPosition(0);
            }
//            if(gamepad1.left_trigger>0){
//                robot.leftArm.setPower(gamepad1.left_trigger*.5);
//            }else{
//                robot.leftArm.setPower(0.1);
//            }
           }
    }
}