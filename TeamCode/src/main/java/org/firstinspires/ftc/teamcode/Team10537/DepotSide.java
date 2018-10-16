package org.firstinspires.ftc.teamcode.Team10537;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * Created by Adam on 11/27/2017.
 */
//@Disabled
@Autonomous(name="DepotSide", group ="Concept")

public class DepotSide extends LinearOpMode {

    public enum GoldLocation {
        UNKNOWN,
        LEFT,
        CENTER,
        RIGHT
    }

    GoldLocation location = GoldLocation.UNKNOWN;

    private GoldAlignDetector detector;
    /* Declare OpMode members. */
    // Use a Pushbot's hardware
    MecanumBasebot robot   = new MecanumBasebot();
    private ElapsedTime runtime = new ElapsedTime();

    static final double     FORWARD_SPEED =  0.5;
    static final double     TURN_SPEED    = 0.25;
    static final double     ticks = 1120*0.89;
    static final double     ticksRevHD = 2240;
    static final double     ticksCoreHex = 288;
    static final int        WHEEL_DIAMETER_INCHES = 4;
    static final int        SPOOL_DIAMETER_INCHES = 1;
    static final double        SHAFT_DIAMETER_INCHES = 0.75;
    static final int        DRIVE_GEAR_REDUCTION = 1;
    static final double     VERTICAL_GEAR_REDUCTION = 2;
    static final int        HORIZONTAL_GEAR_REDUCTION = 1;
    static final int        HAND_GEAR_REDUCTION = 1;
    static final double     PI  = 3.1415;
    static final double     DISTANCE_FOR_90 = (7 * PI / 2) + 1.5;
    static final double COUNTS_PER_INCH =(ticks * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);
    static final double COUNTS_PER_INCH_HORIZONTAL =(ticksRevHD * HORIZONTAL_GEAR_REDUCTION) / (SPOOL_DIAMETER_INCHES * PI);
    static final double COUNTS_PER_INCH_VERTICAL =(ticksCoreHex * VERTICAL_GEAR_REDUCTION) / (SPOOL_DIAMETER_INCHES * PI);
    static final double COUNTS_PER_INCH_HAND =(ticksCoreHex * HAND_GEAR_REDUCTION) / (SHAFT_DIAMETER_INCHES * PI);
    int stage = 1;
    int craterSelection;

    //Setup for the internal imu in the expansion hub
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    @Override public void runOpMode(){
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        //Instantiate the imu and all of it's parameters
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters2.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled      = true;
        parameters2.loggingTag          = "IMU";
        parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters2);
        //Initialize the HarwareBasebot
        robot.init(hardwareMap);
        //Wait for start and show imu data
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        composeTelemetry();

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        telemetry.addData("Front Left ", String.format("number: " + robot.leftFrontMotor.getCurrentPosition()));
        telemetry.addData("Front Right ", String.format("number: " + robot.rightFrontMotor.getCurrentPosition()));
        telemetry.addData("Back Left ", String.format("number: " + robot.leftRearMotor.getCurrentPosition()));
        telemetry.addData("Back Right ", String.format("number: " + robot.rightRearMotor.getCurrentPosition()));
        //Run the autonomous program
        run();

    }

    void run(){
        //deployRobot();
        test();
        alignAndKnock();
        //0:Straight from minerals
        //1:Back to lander around left
        //2:Back to lander around right
        toDepot();
        dropGamePiece();
        sleep2(2);
        //0:Our crater
        //1:Other team's crater
        toSpot(1);
        //0:Straight to left side of crater
        //1:Around minerals to right side of crater
        toCrater(0);
        motorsStop();

    }

    void deployRobot(){
        //moveHand(1,0.5,0);
        robot.handMotor.setPower(0.7);
        sleep2(0.5);
        robot.handMotor.setPower(0);
        //only a little bit
        //moveHorizontal(5,0.5,0);
        //drop to the ground
        moveVertical(1,0.5,0);
        robot.lock.setPosition(0.5);
        robot.handMotor.setPower(-0.5);
        sleep2(0.5);
        robot.handMotor.setPower(0);
    }

    void test(){
        runtime.reset();
        while(!detector.isFound()&&runtime.seconds()<5){
            telemetry.update();
        }
        if(detector.isFound()){
            double x = detector.getXPosition();
            if(x < 200){
                location = GoldLocation.LEFT;
            }else if(x < 450){
                location = GoldLocation.CENTER;
            }else if(x > 450 && x < 640){
                location = GoldLocation.RIGHT;
            }else{
                location = GoldLocation.UNKNOWN;
            }
        }else{
            location = GoldLocation.UNKNOWN;
        }
        detector.disable();
    }
    void alignAndKnock(){
        switch(location){
            case UNKNOWN:
                move(25+18);
                break;
            case LEFT:
                faceToTheLeft(32);
                move(29+18);
                break;
            case CENTER:
                move(25+18);
                break;
            case RIGHT:
                faceToTheRight(32);
                move(29+18);
                break;
            default:
                move(25+18);
                break;
        }
    }
    void toDepot(){
        switch(location){
            case UNKNOWN:
                move(19+18);
                break;
            case LEFT:
                faceToTheRight(51);
                move(24+18);
                break;
            case CENTER:
                move(19+18);
                break;
            case RIGHT:
                faceToTheLeft(51);
                move(24+18);
                break;
            default:
                move(19+18);
        }
    }
    void dropGamePiece(){
        robot.leftHand.setPosition(0);
        robot.rightHand.setPosition(0);
        sleep2(2);
        robot.rightHand.setPosition(0.5);
        robot.leftHand.setPosition(0.5);
    }
    void toSpot(int selection){
        if(selection == 0) {
            faceToTheLeft(45 + 5);
            moveBackward(60);
            craterSelection = 0;
        }else if(selection ==1){
            faceToTheRight(45);
            moveBackward(60);
            craterSelection = 1;
        }
    }
    void toCrater(int selection){
        if(craterSelection == 0) {
            if (selection == 0) {
                moveBackward(30);
            } else if (selection == 0) {
                face0();
                moveBackward(80);
                faceToTheRight(45);
                move(24);
            }
        }else if(craterSelection == 1){
            if(selection ==0){
                moveBackward(30);
            }else if(selection == 1){
                face0();
                moveBackward(80);
                faceToTheRight(45);
                move(24);
            }
        }
    }

    void moveVertical( double inches,double speed, int direction){
        //Intended to move the distance forward in inches passed to the function
        // How far are we to move, in ticks instead of revolutions
        int denc = (int)Math.round(inches * COUNTS_PER_INCH_VERTICAL);
        // Tell the motors where we are going
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int liftDistance;
        if(direction == 0){
            liftDistance = robot.lift.getCurrentPosition() - (denc);
        }else if(direction == 1){
            liftDistance = robot.lift.getCurrentPosition() + (denc);
        }else{
            liftDistance = 0;
        }
        robot.lift.setTargetPosition(liftDistance);
        //Run
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Give them the power level
        robot.lift.setPower(speed);
        //Wait until they are done
        while(robot.lift.isBusy()){
            robot.lift.setPower(speed);
        }
        //Stop the motors
        robot.lift.setPower(0);
    }
    void moveHorizontal( double inches,double speed, int direction){
        //Intended to move the distance forward in inches passed to the function
        // How far are we to move, in ticks instead of revolutions
        int denc = (int)Math.round(inches * COUNTS_PER_INCH_HORIZONTAL);
        // Tell the motors where we are going
        robot.push.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int pushDistance;
        if(direction == 0){
            pushDistance = robot.push.getCurrentPosition() + (denc);
        }else if(direction == 1){
            pushDistance = robot.push.getCurrentPosition() - (denc);
        }else{
            pushDistance = 0;
        }
        robot.push.setTargetPosition(pushDistance);
        //Run
        robot.push.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Give them the power level
        robot.push.setPower(speed);
        //Wait until they are done
        while(robot.push.isBusy()){
            robot.push.setPower(speed);
        }
        //Stop the motors
        robot.push.setPower(0);
    }
    void moveHand( double inches,double speed, int direction){

        //Intended to move the distance forward in inches passed to the function
        // How far are we to move, in ticks instead of revolutions
        int denc = (int)Math.round(inches * COUNTS_PER_INCH_HORIZONTAL);
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
    void motorsForward(){
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftFrontMotor.setPower(FORWARD_SPEED);
        robot.rightFrontMotor.setPower(FORWARD_SPEED);
        robot.leftRearMotor.setPower(FORWARD_SPEED);
        robot.rightRearMotor.setPower(FORWARD_SPEED);
    }
    void motorsStop(){
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.leftRearMotor.setPower(0);
        robot.rightRearMotor.setPower(0);
    }
    void moveMotors(double leftF, double rightF, double leftR, double rightR, double inches,double speed){
        //Intended to move the distance forward in inches passed to the function
        // How far are we to move, in ticks instead of revolutions
        int denc = (int)Math.round(inches * COUNTS_PER_INCH);
        // Tell the motors where we are going
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int leftFront = robot.leftFrontMotor.getCurrentPosition() + (int)(denc*leftF);
        int rightFront = robot.rightFrontMotor.getCurrentPosition() + (int)(denc*rightF);
        int leftRear = robot.leftRearMotor.getCurrentPosition() + (int)(denc*leftR);
        int rightRear = robot.rightRearMotor.getCurrentPosition() + (int)(denc*rightR);
        robot.leftFrontMotor.setTargetPosition(leftFront);
        robot.rightFrontMotor.setTargetPosition(rightFront);
        robot.leftRearMotor.setTargetPosition(leftRear);
        robot.rightRearMotor.setTargetPosition(rightRear);
        //Run
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Give them the power level
        robot.leftFrontMotor.setPower(speed);
        robot.rightFrontMotor.setPower(speed);
        robot.leftRearMotor.setPower(speed);
        robot.rightRearMotor.setPower(speed);
        //Wait until they are done
        double startTime = runtime.seconds();
        while((robot.leftFrontMotor.isBusy() ||robot.rightFrontMotor.isBusy() || robot.leftRearMotor.isBusy() || robot.rightRearMotor.isBusy())&&(runtime.seconds()-startTime)<5){
            robot.leftFrontMotor.setPower(speed);
            robot.rightFrontMotor.setPower(speed);
            robot.leftRearMotor.setPower(speed);
            robot.rightRearMotor.setPower(speed);
        }
        //Stop the motors
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.leftRearMotor.setPower(0);
        robot.rightRearMotor.setPower(0);
    }
    void move(double inches){
        moveMotors(1,1,1,1,inches,FORWARD_SPEED);
    }
    void moveBackward(double inches){
        moveMotors(-1,-1,-1,-1,inches, FORWARD_SPEED);
    }
    void turnRight(double inches){
        moveMotors(1,-1,1,-1,inches, TURN_SPEED);
    }
    void faceLeft(){
        //Face 90 degrees left of the starting direction
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle<82.5 && runtime.seconds() < 5) {
            robot.leftFrontMotor.setPower(-TURN_SPEED);
            robot.rightFrontMotor.setPower(TURN_SPEED);
            robot.leftRearMotor.setPower(-TURN_SPEED);
            robot.rightRearMotor.setPower(TURN_SPEED);
        }
        robot.rightFrontMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.rightRearMotor.setPower(0);
        robot.leftRearMotor.setPower(0);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
        void faceRight(){
        //Face 90 degrees right of the starting direction
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle>-82.5 && runtime.seconds() < 5) {
            robot.leftFrontMotor.setPower(TURN_SPEED);
            robot.rightFrontMotor.setPower(-TURN_SPEED);
            robot.leftRearMotor.setPower(TURN_SPEED);
            robot.rightRearMotor.setPower(-TURN_SPEED);
        }
        if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle>-82.5 && runtime.seconds() < 5){
            while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle>-82.5) {
                robot.leftFrontMotor.setPower(TURN_SPEED+0.1);
                robot.rightFrontMotor.setPower(-TURN_SPEED+0.1);
                robot.leftRearMotor.setPower(TURN_SPEED+0.1);
                robot.rightRearMotor.setPower(-TURN_SPEED+0.1);
            }
        }
        robot.rightFrontMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.rightRearMotor.setPower(0);
        robot.leftRearMotor.setPower(0);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    void face0(){
        //Return the to facing the original direction the started
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot. rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        float angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        runtime.reset();
        if(angle > 0) {
            while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > 7.5&& runtime.seconds() < 5) {
                robot.leftFrontMotor.setPower(TURN_SPEED);
                robot.rightFrontMotor.setPower(-TURN_SPEED);
                robot.leftRearMotor.setPower(TURN_SPEED);
                robot.rightRearMotor.setPower(-TURN_SPEED);
            }
        }else if(angle < 0){
            while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < -7.5 && runtime.seconds() < 5) {
                robot.leftFrontMotor.setPower(-TURN_SPEED);
                robot.rightFrontMotor.setPower(TURN_SPEED);
                robot.leftRearMotor.setPower(-TURN_SPEED);
                robot.rightRearMotor.setPower(TURN_SPEED);
            }
        }
        robot.rightFrontMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.rightRearMotor.setPower(0);
        robot.leftRearMotor.setPower(0);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    void faceBackward(){
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle<172.5 && runtime.seconds() < 5) {
            robot.leftFrontMotor.setPower(TURN_SPEED);
            robot.rightFrontMotor.setPower(-TURN_SPEED);
            robot.leftRearMotor.setPower(TURN_SPEED);
            robot.rightRearMotor.setPower(-TURN_SPEED);
        }
        robot.rightFrontMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.rightRearMotor.setPower(0);
        robot.leftRearMotor.setPower(0);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    void faceToTheRight(int degrees){
        //Face 90 degrees right of the starting direction
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle>((-degrees)+7.5) && runtime.seconds() < 5) {
            robot.leftFrontMotor.setPower(TURN_SPEED);
            robot.rightFrontMotor.setPower(-TURN_SPEED);
            robot.leftRearMotor.setPower(TURN_SPEED);
            robot.rightRearMotor.setPower(-TURN_SPEED);
        }
        if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle>((-degrees)+7.5) && runtime.seconds() < 5){
            while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle>((-degrees)+7.5)) {
                robot.leftFrontMotor.setPower(TURN_SPEED+0.1);
                robot.rightFrontMotor.setPower(-TURN_SPEED+0.1);
                robot.leftRearMotor.setPower(TURN_SPEED+0.1);
                robot.rightRearMotor.setPower(-TURN_SPEED+0.1);
            }
        }
        robot.rightFrontMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.rightRearMotor.setPower(0);
        robot.leftRearMotor.setPower(0);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    void faceToTheLeft(int degrees){
        //Face 90 degrees left of the starting direction
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle<(degrees-7.5) && runtime.seconds() < 5) {
            robot.leftFrontMotor.setPower(-TURN_SPEED);
            robot.rightFrontMotor.setPower(TURN_SPEED);
            robot.leftRearMotor.setPower(-TURN_SPEED);
            robot.rightRearMotor.setPower(TURN_SPEED);
        }
        robot.rightFrontMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.rightRearMotor.setPower(0);
        robot.leftRearMotor.setPower(0);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    void turnLeft(double inches){
        moveMotors(-1,1,-1,1,inches,TURN_SPEED);
    }
    void strafeLeft(double inches){
        moveMotors(-1,1,1,-1,inches*2,TURN_SPEED);
    }
    void strafeRight(double inches){
        moveMotors(1,-1,-1,1,inches*2,TURN_SPEED);
    }
    void sleep2(double seconds){
        double startTime = runtime.seconds();
        while ((runtime.seconds()-startTime) < seconds) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    void composeTelemetry() {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });
        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });
        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}