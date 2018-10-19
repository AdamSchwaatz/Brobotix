package org.firstinspires.ftc.teamcode.Team10537;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
    static final double     liftSpeed = 0.5;
    static final double     pushSpeed = 0.5;
    static final double     handSpeed = 0.5;
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
    public boolean handGo = false;

    public enum Status {
        Starting,
        Lifting,
        Pushing,
        Hand,
        LP,
        LH,
        PH,
        LPH
    }

    Status status = Status.Starting;

    @Override
    public void runOpMode() throws InterruptedException {
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
        Status status = Status.Starting;


        while(opModeIsActive()){
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Status", "Gamepad1: " + gamepad1.toString());
            telemetry.addData("Status","Status: " + status);
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


            while (status == Status.Starting) {
                driving();
            }
            while(status == Status.Lifting){
                driving();
                robot.lift.setPower(liftSpeed);
                if(!robot.lift.isBusy()){
                    liftGo = false;
                }
            }
            while(status == Status.Pushing){
                driving();
                robot.push.setPower(pushSpeed);
                if(!robot.push.isBusy()){
                    pushGo = false;
                }
            }
            while(status == Status.Hand){
                driving();
                robot.handMotor.setPower(handSpeed);
                if(!robot.handMotor.isBusy()){
                    handGo = false;
                }
            }
            while(status == Status.LPH){
                driving();
                robot.lift.setPower(liftSpeed);
                robot.push.setPower(pushSpeed);
                robot.handMotor.setPower(handSpeed);
                if(!robot.lift.isBusy()){
                    liftGo = false;
                }
                if(!robot.push.isBusy()){
                    pushGo = false;
                }
                if(!robot.handMotor.isBusy()){
                    handGo = false;
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
        }
        robot.lift.setPower(0);
        robot.push.setPower(0);
        robot.handMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.leftRearMotor.setPower(0);
        robot.rightRearMotor.setPower(0);
    }

    void checkStatus(){
        if(liftGo&&pushGo&&handGo){
            status = Status.LPH;
        }else if(liftGo&&pushGo){
            status = Status.LP;
        }else if(pushGo&&handGo){
            status = Status.PH;
        }else if(liftGo&&handGo){
            status = Status.LH;
        }else if(liftGo){
            status = Status.Lifting;
        }else if(pushGo){
            status = Status.Pushing;
        }else if(handGo){
            status = Status.Hand;
        }else{
            status = Status.Starting;
        }
    }


    void driving(){
        checkStatus();
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
        if(!(status == Status.Lifting||status == Status.LH||status == Status.LP||status == Status.LPH)){
            robot.lift.setPower(gamepad2.left_stick_y);
        }
        if(!(status == Status.Hand||status == Status.LH||status == Status.LPH||status == Status.PH)){
            robot.handMotor.setPower(gamepad2.right_stick_y);
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

        //Unlock
        if (gamepad1.y) {
            robot.lock.setPosition(0);
        }
        if (gamepad1.a) {
            robot.lock.setPosition(0.5);
        }
//
//        //Push
//        while (gamepad1.dpad_up) {
//            robot.push.setPower(0.5);
//        }
//        if (!(gamepad1.dpad_up || gamepad1.dpad_down)) {
//            robot.push.setPower(0);
//        }
//        while (gamepad1.dpad_down) {
//            robot.push.setPower(-0.2);
//        }

        //Lift
        if(gamepad2.y){
            if ((runtime.seconds() - liftTime) > 1) {
                if (liftPosition == 0) {
                    liftGo = true;
                    //9.5
                    moveVertical(9, 0);

                    liftPosition = 1;
                    liftTime = runtime.seconds();

                } else if (liftPosition == 1) {
                    liftGo = true;
                    //9.5
                    moveVertical(9, 1);
                    liftPosition = 0;
                    liftTime = runtime.seconds();
                }
            }
        }

        //Push Arm
        if(gamepad2.x){
            if((runtime.seconds()-pushTime)>1){
                if(pushPosition == 0){
                    pushGo = true;
                    moveHorizontal(6, 0);
                    pushPosition = 1;
                    pushTime = runtime.seconds();
                }else if(pushPosition == 1){
                    pushGo = true;
                    moveHorizontal(6, 1);
                    pushPosition = 0;
                    pushTime = runtime.seconds();
                }
            }
        }

    }

    void moveVertical( double inches, int direction){
        //Intended to move the distance forward in inches passed to the function
        // How far are we to move, in ticks instead of revolutions
        int denc = (int)Math.round(inches * COUNTS_PER_INCH_VERTICAL);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Tell the motors where we are going
        int liftDistance;
        if(direction == 0){
            liftDistance = robot.lift.getCurrentPosition() - (denc);
        }else if(direction == 1){
            liftDistance = robot.lift.getCurrentPosition() + (denc);
//            liftDistance = 0;
        }else{
            liftDistance = 0;
        }
        robot.lift.setTargetPosition(liftDistance);
        //Run
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(liftSpeed);
        status = Status.Lifting;
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
    void moveHorizontal( double inches, int direction){
        //Intended to move the distance forward in inches passed to the function
        // How far are we to move, in ticks instead of revolutions
        int denc = (int)Math.round(inches * COUNTS_PER_INCH_HORIZONTAL);
        // Tell the motors where we are going
//        robot.push.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        robot.push.setPower(pushSpeed);
        //Wait until they are done
//        while(robot.push.isBusy()){
//            robot.push.setPower(speed);
//        }
        //Stop the motors
//        robot.push.setPower(0);
    }
    void moveHand( double inches, int direction){

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
        robot.handMotor.setPower(handSpeed);
        //Wait until they are done
        while(robot.handMotor.isBusy()){
            robot.handMotor.setPower(handSpeed);
        }
        //Stop the motors
        robot.handMotor.setPower(0);
    }
}