package org.firstinspires.ftc.teamcode.Team10537;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Adam on 2/17/2018.
 */

@TeleOp(name="Mecanum Drive", group="Adam")

public class MecanumDrive extends LinearOpMode {

    private ElapsedTime runtime  = new ElapsedTime();
    MecanumBasebot robot    = new MecanumBasebot();   // Use a Pushbot's hardware

    static final double     FORWARD_SPEED =  0.5;
    static final double     TURN_SPEED    = 0.25;
    static final double     ticksRevHD = 2240;
    static final double     ticksCoreHex = 288;
    static final int        WHEEL_DIAMETER_INCHES = 1;
    static final double     SHAFT_DIAMETER_INCHES = 0.75;
    static final double     VERTICAL_GEAR_REDUCTION = 2;
    static final int        HORIZONTAL_GEAR_REDUCTION = 1;
    static final int        HAND_GEAR_REDUCTION = 1;
    static final double     PI  = 3.1415;
    static final double COUNTS_PER_INCH_HORIZONTAL =(ticksRevHD * HORIZONTAL_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);
    static final double COUNTS_PER_INCH_VERTICAL =(ticksCoreHex * VERTICAL_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);
    static final double COUNTS_PER_INCH_HAND =(ticksCoreHex * HAND_GEAR_REDUCTION) / (SHAFT_DIAMETER_INCHES * PI);
    public double buttonTime = 0.3;
    public double dumpTime = 0;
    public double liftTime = 0;
    public double spinTime = 0;
    public double pushTime = 0;
    public double dropTime = 0;
    public int dumpPosition = 0;
    public int liftPosition = 0;
    public int spinDirection = 0;
    public int pushPosition= 0;
    public int dropPosition = 0;
    public boolean pushGo = false;
    public boolean liftGo = false;
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.handMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.push.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.handMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Status", "Controller" + gamepad1.toString());
            telemetry.update();

            //Wheel Controls

            //Version 2.3 Driver Controls

//            Gamepad 1       Gampepad 2
//            x dump          x spinners-clock-counter-stop
//            y unlock        y grabber-out-in
//            a               a lift-up-down
//            b               b
//            leftJoystick    leftJoystick
//            x drive         x
//            y drive         y lift
//            rightJoystick   rightJoystick
//            x turn          x
//            y turn          y grabber
//            d-pad           d-pad
//            up push-out     up
//            down push-in    down
//            left            left
//            right           right




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

            robot.lift.setPower(gamepad2.left_stick_y);
            robot.handMotor.setPower(gamepad2.right_stick_y);

            while(gamepad1.dpad_up){
                robot.push.setPower(0.5);
            }
            if(!(gamepad1.dpad_up||gamepad1.dpad_down)){
                robot.push.setPower(0);
            }
            while(gamepad1.dpad_down){
                robot.push.setPower(-0.2);
            }


            //Unlock
            if(gamepad1.y){
                robot.lock.setPosition(0);
            }
            if(gamepad1.a){
                robot.lock.setPosition(0.5);
            }

            //Spinners
            if(gamepad1.x){
                if((runtime.seconds()-spinTime)>buttonTime){
                    if(spinDirection == 0){
                        robot.rightHand.setPosition(1);
                        robot.leftHand.setPosition(1);
                        spinDirection = 1;
                        spinTime = runtime.seconds();
                    }else if(spinDirection == 1){
                        robot.rightHand.setPosition(0);
                        robot.leftHand.setPosition(0);
                        spinDirection = 2;
                        spinTime = runtime.seconds();
                    }else if(spinDirection == 2){
                        robot.rightHand.setPosition(0.5);
                        robot.leftHand.setPosition(0.5);
                        spinDirection = 0;
                    }
                }
            }
            //Push Arm
            if(gamepad2.x){
                if((runtime.seconds()-pushTime)>1){
                    if(pushPosition == 0){
                        moveHorizontal(6, 0.5, 0);
                        pushPosition = 1;
                        pushTime = runtime.seconds();
                    }else if(pushPosition == 1){
                        moveHorizontal(6, 0.4, 1);
                        pushPosition = 0;
                        pushTime = runtime.seconds();
                    }
                }
            }
            //Hand Drop
//            if(gamepad2.b){
//                if((runtime.seconds()-dropTime)>1){
//                    if(dropPosition == 0){
//                        moveHand(5, 0.5,0);
//                        liftPosition = 1;
//                        liftTime = runtime.seconds();
//                    }else if(liftPosition == 1){
//                        //9.5
//                        moveHand(5, -0.5,1);
//                        liftPosition = 0;
//                        liftTime = runtime.seconds();
//                    }
//                }
//            }
            //Lift Arm
            if(gamepad2.y){
                if((runtime.seconds()-liftTime)>1){
                    if(liftPosition == 0){
                        //9.5
                        moveVertical(9, 0.5, 0);
                        liftPosition = 1;
                        liftTime = runtime.seconds();
                        liftGo = true;
                    }else if(liftPosition == 1){
                        //9.5
                        moveVertical(9, 0.5, 1);
                        liftPosition = 0;
                        liftTime = runtime.seconds();
                    }
                }
            }
//            if(robot.lift.isBusy()&&(runtime.seconds()-liftTime)>1){
//                liftGo = true;
//            }else{
//                liftGo = false;
//            }
//            if(liftGo){
//                robot.lift.setPower(0.5);
//            }else{
//                robot.lift.setPower(0);
//            }
            //Dump Hand
            if(gamepad1.b){
                if((runtime.seconds()-dumpTime)>buttonTime){
                    if(dumpPosition == 0){
                        robot.dumpHand.setPosition(1);
                        dumpPosition = 1;
                        dumpTime = runtime.seconds();
                    }else if(dumpPosition == 1){
                        robot.dumpHand.setPosition(0);
                        dumpPosition = 0;
                        dumpTime = runtime.seconds();
                    }
                }
            }
           }
        robot.lift.setPower(0);
        robot.push.setPower(0);
        robot.handMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.leftRearMotor.setPower(0);
        robot.rightRearMotor.setPower(0);
    }
    void moveVertical( double inches,double speed, int direction){
        //Intended to move the distance forward in inches passed to the function
        // How far are we to move, in ticks instead of revolutions
        int denc = (int)Math.round(inches * COUNTS_PER_INCH_VERTICAL);
        // Tell the motors where we are going
        int liftDistance;
        if(direction == 0){
            liftDistance = robot.lift.getCurrentPosition() - (denc);
        }else if(direction == 1){
//            liftDistance = robot.lift.getCurrentPosition() + (denc);
            liftDistance = 0;
        }else{
            liftDistance = 0;
        }
        robot.lift.setTargetPosition(liftDistance);
        //Run
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //liftGo = true;
        // Give them the power level
//        robot.lift.setPower(speed);
//        //Wait until they are done
//        while(robot.lift.isBusy()){
//            robot.lift.setPower(speed);
//        }
//        //Stop the motors
//        robot.lift.setPower(0);
    }
    void moveHorizontal( double inches,double speed, int direction){
        //Intended to move the distance forward in inches passed to the function
        // How far are we to move, in ticks instead of revolutions
        int denc = (int)Math.round(inches * COUNTS_PER_INCH_HORIZONTAL);
        // Tell the motors where we are going
        robot.push.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int pushDistance;
        if(direction == 0){
            pushDistance = (denc);
        }else if(direction == 1){
            pushDistance = 0;
        }else{
            pushDistance = 0;
        }
        robot.push.setTargetPosition(pushDistance);
        //Run
        robot.push.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Give them the power level
        robot.push.setPower(speed);
        //Wait until they are done
//        while(robot.push.isBusy()){
//            robot.push.setPower(speed);
//        }
        //Stop the motors
//        robot.push.setPower(0);
    }
    void moveHand( double inches,double speed, int direction){

        //Intended to move the distance forward in inches passed to the function
        // How far are we to move, in ticks instead of revolutions
        int denc = (int)Math.round(inches * COUNTS_PER_INCH_HAND);
        // Tell the motors where we are going
        robot.handMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int spinDistance;
        if(direction == 0){
            spinDistance = robot.handMotor.getCurrentPosition() - (denc);
        }else if(direction == 1){
            spinDistance = robot.handMotor.getCurrentPosition() + (denc);
        }else{
            spinDistance = 0;
        }
        robot.handMotor.setTargetPosition(spinDistance);
        //Run
        robot.handMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Give them the power level
        robot.handMotor.setPower(speed);
        //Wait until they are done
        while(robot.handMotor.isBusy()){
            robot.handMotor.setPower(speed);
        }
        //Stop the motors
        robot.handMotor.setPower(0);
    }
}