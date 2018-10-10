package org.firstinspires.ftc.teamcode.Examples.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Examples.HardwareBasebot;

import java.util.Locale;

/**
 * Created by Adam on 11/27/2017.
 */
//@Disabled
@Autonomous(name="TestDistance", group ="Concept")
@Disabled
public class TestDistance extends LinearOpMode {

    /* Declare OpMode members. */
    // Use a Pushbot's hardware
    HardwareBasebot robot   = new HardwareBasebot();
    private ElapsedTime runtime = new ElapsedTime();

    //Values for Distances for the Encoders
    static final double     FORWARD_SPEED =  0.15;
    static final double     TURN_SPEED    = 0.25;
    static final int        ticks = 1120/2;
    static final int        WHEEL_DIAMETER_INCHES = 4;
    static final int        DRIVE_GEAR_REDUCTION = 1;
    static final double     PI  = 3.1415;
    static final double     DISTANCE_FOR_90 = (7 * PI / 2) + 1.5;
    static final double COUNTS_PER_INCH =(ticks * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);
    static final int left = 20;
    static final int middle = 7;
    static final int right = 4;
    static double variable = 0;

    //Setup for Vuforia
    VuforiaLocalizer vuforia;
    int cameraMonitorViewId;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    RelicRecoveryVuMark vuMark;
    //Setup for the internal imu in the expansion hub
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    @Override public void runOpMode() {
        //Setup the camera for Vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //Key required to make Vuforia run
        parameters.vuforiaLicenseKey = "AbJZaj3/////AAAAGXVXrm+sFEbdtdCk8BOxyOU9PPL+qLI/Wke/rc/l5ILyKhpZX8x2Rdxf0limK8ud/GmOBLYLVVPy6rk/aD2eJJ0DLCFk1H+Qv3Jp8hkDrUOjD+BKb1HU2jaH7uiHBxlMEAmoBrRTOWG3t2HCdJrTssr3mnewGyVtF8XyCykNUZkZdOP3P65l7CwLDrNwcuY3rmTKh/tf7CU4VLrfFgrH26VNbk4UNViqJGF6OsRxLxZwSHyOxry3eSGcaAuW7e+osKAdXa95/7wdiqYPs4DmfO/EmFDm//8xQR0m6Lz/lA2dwwKmFWPbitBaYllsKd1DSQVUCIpJIFX6YxHRPOGVMNYMOak3t+uouWgD4phOMMWT";
        //Tell it to use the rear camera on the phone
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        //Use the Relic Recovery Vumark Template for Vuforia
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
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
        //Set the motors to the right direction and to use encoders
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.leftMotor.setDirection(DcMotor.Direction.FORWARD);
        //Wait for start and show imu data
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        composeTelemetry();
        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        relicTrackables.activate();
        //Search for the Vumark Pictures
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        boolean run = false;
        runtime.reset();
        while (vuMark == RelicRecoveryVuMark.UNKNOWN&&runtime.seconds() < 1)
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", vuMark);
            run = true;
        }else {
            telemetry.addData("VuMark", "not visible");
        }
        if(!run){
            vuMark = RelicRecoveryVuMark.CENTER;
        }
        if(vuMark == RelicRecoveryVuMark.LEFT){
            variable = (int)Math.round(4 * COUNTS_PER_INCH);
        }else if(vuMark == RelicRecoveryVuMark.CENTER){
            variable = (int)Math.round(8 * COUNTS_PER_INCH);
        }else if(vuMark == RelicRecoveryVuMark.RIGHT){
            variable = (int)Math.round(12 * COUNTS_PER_INCH);
        }else{
            variable = (int)Math.round(8 * COUNTS_PER_INCH);
        }
        telemetry.update();
        //Run the autonomous program
        run();
    }
    void run(){
        move(24);
        face0();
        move(variable);
        faceLeft();
        move(4);
        moveBackward(2);
    }
    void move(double inches){
        //Intended to move the distance forward in inches passed to the function
        // How far are we to move, in ticks instead of revolutions?
        int denc = (int)Math.round(inches * COUNTS_PER_INCH);
        // Tell the motors where we are going
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int left = robot.leftMotor.getCurrentPosition() + denc;
        int right = robot.rightMotor.getCurrentPosition() + denc;
        robot.leftMotor.setTargetPosition(left);
        robot.rightMotor.setTargetPosition(right);
        //Run
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Give them the power level
        robot.leftMotor.setPower(FORWARD_SPEED);
        robot.rightMotor.setPower(FORWARD_SPEED);
        // Wait until they are done
        runtime.reset();
        while(opModeIsActive()&&(robot.leftMotor.isBusy() || robot.rightMotor.isBusy())&&runtime.seconds() < 5){
            telemetry.update();
            robot.leftMotor.setPower(FORWARD_SPEED);
            robot.rightMotor.setPower(FORWARD_SPEED);
        }
        //Stop the motors
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        telemetry.update();
    }
    void moveBackward(double inches){
        //Intended to move the distance backward in inches passed to the function
        // How far are we to move, in ticks instead of revolutions?
        int denc = (int)Math.round(inches * COUNTS_PER_INCH);
        // Tell the motors where we are going
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int left = robot.leftMotor.getCurrentPosition() - denc;
        int right = robot.rightMotor.getCurrentPosition() - denc;
        robot.leftMotor.setTargetPosition(left);
        robot.rightMotor.setTargetPosition(right);
        //Run
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Give them the power level
        robot.leftMotor.setPower(FORWARD_SPEED);
        robot.rightMotor.setPower(FORWARD_SPEED);
        // Wait until they are done
        runtime.reset();
        while(opModeIsActive()&&(robot.leftMotor.isBusy() || robot.rightMotor.isBusy())&&runtime.seconds() < 2){
            telemetry.update();
            robot.leftMotor.setPower(FORWARD_SPEED);
            robot.rightMotor.setPower(FORWARD_SPEED);
        }
        //Stop the motors
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        telemetry.update();
    }
    void turnRight(double inches){
        //Turn Right the amount of inches passed to the function
        int denc = (int)Math.round(inches * COUNTS_PER_INCH);
        // Tell the motors where we are going
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int left = robot.leftMotor.getCurrentPosition() + denc;
        int right = robot.rightMotor.getCurrentPosition() - denc;
        robot.leftMotor.setTargetPosition(left);
        robot.rightMotor.setTargetPosition(right);
        //Run
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Give them the power
        robot.leftMotor.setPower(TURN_SPEED);
        robot.rightMotor.setPower(TURN_SPEED);
        // Wait until they are done
        runtime.reset();
        while(opModeIsActive()&&(robot.leftMotor.isBusy() || robot.rightMotor.isBusy())&&runtime.seconds() < 2){
            telemetry.update();
            robot.leftMotor.setPower(FORWARD_SPEED);
            robot.rightMotor.setPower(FORWARD_SPEED);
        }
        //Stop the motors
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        telemetry.update();
    }
    void faceLeft(){
        //Face 90 degrees left of the starting direction
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle<82.5 && runtime.seconds() < 5) {
            robot.leftMotor.setPower(-TURN_SPEED);
            robot.rightMotor.setPower(TURN_SPEED);
        }
        robot.rightMotor.setPower(0);
        robot.leftMotor.setPower(0);
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    void faceRight(){
        //Face 90 degrees right of the starting direction
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle>-82.5 && runtime.seconds() < 5) {
            robot.leftMotor.setPower(TURN_SPEED);
            robot.rightMotor.setPower(-TURN_SPEED);
        }
        if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle>-82.5 && runtime.seconds() < 5){
            while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle>-82.5) {
                robot.leftMotor.setPower(TURN_SPEED+0.1);
                robot.rightMotor.setPower(-TURN_SPEED+0.1);
            }
        }
        robot.rightMotor.setPower(0);
        robot.leftMotor.setPower(0);
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    void face0(){
        //Return the robot to facing the original direction the robot started
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        float angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        runtime.reset();
        if(angle > 0) {
            while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > 7.5&& runtime.seconds() < 5) {
                robot.leftMotor.setPower(TURN_SPEED);
                robot.rightMotor.setPower(-TURN_SPEED);
            }
        }else if(angle < 0){
            while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < -7.5 && runtime.seconds() < 5) {
                robot.leftMotor.setPower(-TURN_SPEED);
                robot.rightMotor.setPower(TURN_SPEED);
            }
        }
        robot.rightMotor.setPower(0);
        robot.leftMotor.setPower(0);
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    void faceBackward(){
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle<172.5 && runtime.seconds() < 5) {
            robot.leftMotor.setPower(TURN_SPEED);
            robot.rightMotor.setPower(-TURN_SPEED);
        }
        robot.rightMotor.setPower(0);
        robot.leftMotor.setPower(0);
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    void turnLeft(double inches){
        int denc = (int)Math.round(inches * COUNTS_PER_INCH);
        // Tell the motors where we are going
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int left = robot.leftMotor.getCurrentPosition() - denc;
        int right = robot.rightMotor.getCurrentPosition() + denc;
        robot.leftMotor.setTargetPosition(left);
        robot.rightMotor.setTargetPosition(right);
        //Run
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Give them the power level
        robot.leftMotor.setPower(TURN_SPEED);
        robot.rightMotor.setPower(TURN_SPEED);
        // Wait until they are done
        runtime.reset();
        while(opModeIsActive()&&(robot.leftMotor.isBusy() || robot.rightMotor.isBusy())&&runtime.seconds() < 2){
            telemetry.update();
            robot.leftMotor.setPower(FORWARD_SPEED);
            robot.rightMotor.setPower(FORWARD_SPEED);
        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        telemetry.update();
    }
    public void sleep2(double seconds){
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
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