/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.Examples.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Examples.HardwareBasebot;

//@Disabled
@TeleOp(name="Third Times the Charm", group="Adam")
@Disabled
public class OpMode_TankDrive extends LinearOpMode {

    private ElapsedTime    runtime  = new ElapsedTime();
    HardwareBasebot robot    = new HardwareBasebot();   // Use a Pushbot's hardware

    public final double SPEED = -0.4;
    public final double SPEED_1 = -0.25;
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
            //telemetry.addData("Status", "Hand Rotation" + robot.handRotation);
            telemetry.update();


            //Wheel Controls


            //Version 2.3 Driver Controls



            //Precise
            if(gamepad1.left_trigger > 0){
                if(gamepad1.right_stick_x < 0 && gamepad1.left_stick_y < 0) {   //Forward Left
                        robot.rightMotor.setPower(-gamepad1.left_stick_y*SPEED_1);
                        robot.leftMotor.setPower(((gamepad1.right_stick_x)*2)*(gamepad1.left_stick_y*SPEED_1));
                } else if(gamepad1.right_stick_x > 0 && gamepad1.left_stick_y < 0) {    //Forward Right
                        robot.leftMotor.setPower(-gamepad1.left_stick_y*SPEED_1);
                        robot.rightMotor.setPower(((0.5-gamepad1.right_stick_x)*2)*(-gamepad1.left_stick_y*SPEED_1));
                } else if(gamepad1.right_stick_x < 0 && gamepad1.left_stick_y > 0) {   //Backward Left
                        robot.rightMotor.setPower(-gamepad1.left_stick_y*SPEED_1);
                        robot.leftMotor.setPower(((gamepad1.right_stick_x)*2)*(gamepad1.left_stick_y*SPEED_1));
                } else if(gamepad1.right_stick_x > 0 && gamepad1.left_stick_y > 0) {    //Backward Right
                        robot.leftMotor.setPower(-gamepad1.left_stick_y*SPEED_1);
                        robot.rightMotor.setPower(((gamepad1.right_stick_x-0.5)*2)*(gamepad1.left_stick_y*SPEED_1));
                } else {                                    //Straight
                    robot.leftMotor.setPower(-gamepad1.left_stick_y*SPEED_1);
                    robot.rightMotor.setPower(-gamepad1.left_stick_y*SPEED_1);
                }
                //Normal
            }else{
                if(gamepad1.right_stick_x < 0 && gamepad1.left_stick_y < 0) {   //Forward Left
                    if(gamepad1.right_stick_x>-0.5) {
                        robot.rightMotor.setPower(-gamepad1.left_stick_y*SPEED);
                        robot.leftMotor.setPower(((gamepad1.right_stick_x)*2)*(gamepad1.left_stick_y*SPEED));
                    } else if(gamepad1.right_stick_x <= -0.5) {
                        robot.rightMotor.setPower(-gamepad1.left_stick_y*SPEED);
                        robot.leftMotor.setPower(((0.5 + gamepad1.right_stick_x) * 2) * (-gamepad1.left_stick_y*SPEED));
                    }
                } else if(gamepad1.right_stick_x > 0 && gamepad1.left_stick_y < 0) {    //Forward Right
                    if(gamepad1.right_stick_x<=0.5) {
                        robot.leftMotor.setPower(-gamepad1.left_stick_y*SPEED);
                        robot.rightMotor.setPower(((0.5-gamepad1.right_stick_x)*2)*(-gamepad1.left_stick_y*SPEED));
                    } else if(gamepad1.right_stick_x>0.5) {
                        robot.leftMotor.setPower(-gamepad1.left_stick_y*SPEED);
                        robot.rightMotor.setPower(((gamepad1.right_stick_x-0.5)*2)*(gamepad1.left_stick_y*SPEED));
                    }
                } else if(gamepad1.right_stick_x < 0 && gamepad1.left_stick_y > 0) {   //Backward Left
                    if(gamepad1.right_stick_x>-0.5) {
                        robot.rightMotor.setPower(-gamepad1.left_stick_y*SPEED);
                        robot.leftMotor.setPower(((gamepad1.right_stick_x)*2)*(gamepad1.left_stick_y*SPEED));
                    } else if(gamepad1.right_stick_x <= -0.5) {
                        robot.rightMotor.setPower(-gamepad1.left_stick_y*SPEED);
                        robot.leftMotor.setPower(((0.5 + gamepad1.right_stick_x) * 2) * (-gamepad1.left_stick_y*SPEED));
                    }
                } else if(gamepad1.right_stick_x > 0 && gamepad1.left_stick_y > 0) {    //Backward Right
                    if(gamepad1.right_stick_x<=0.5) {
                        robot.leftMotor.setPower(-gamepad1.left_stick_y*SPEED);
                        robot.rightMotor.setPower(((0.5-gamepad1.right_stick_x)*2)*(-gamepad1.left_stick_y*SPEED));
                    } else if(gamepad1.right_stick_x>0.5) {
                        robot.leftMotor.setPower(-gamepad1.left_stick_y*SPEED);
                        robot.rightMotor.setPower(((gamepad1.right_stick_x-0.5)*2)*(gamepad1.left_stick_y*SPEED));
                    }
                } else {                                    //Straight
                    robot.leftMotor.setPower(-gamepad1.left_stick_y*SPEED);
                    robot.rightMotor.setPower(-gamepad1.left_stick_y*SPEED);
                }
            }

            //Arm Controls

            //Arm
            //Arm Controls
            //Servo Controls
            if(gamepad1.a){
                robot.claw.setPosition(.45);
                robot.claw2.setPosition(.75);
            }
            else if (gamepad1.b) {
                robot.claw.setPosition(.65);
                robot.claw2.setPosition(.55);
            }

            if(gamepad1.x) {
                robot.relic.setPosition(.55);
                robot.leftMotor.setPower(-.25);
                robot.rightMotor.setPower(-.25);
                runtime.reset();
                while(runtime.seconds()>.1){}
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
            }
            if(gamepad1.y)
                robot.relic.setPosition(.35);

            //Arm Controls
//            robot.arm.setPower(gamepad1.right_trigger);
//            robot.arm.setPower(-gamepad1.left_trigger);

        }
    }
}