package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static java.lang.Double.isNaN;

/**
 * Created by User on 10/17/2017.
 */

public class Chassis {

    //create drive train objects
    DcMotor frontLeftDrive;
    DcMotor backLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backRightDrive;

    DcMotor lift;

    Servo depositor;
    Servo markerHolder;

    boolean vision=false;

    //init other objects
    HardwareMap hwm;

    VuforiaLocalizer vuforia;

    DistanceSensor distance;

    ColorSensor color;

    //set final variables
    final static double INIT = 0.0;
    final static double UP = 0.4;

    //constructors
    Chassis() {}
    Chassis(boolean v) {vision=v;}

    //hardware init method
    public void init(HardwareMap inhwm) {
        hwm = inhwm;

        //connect to hardware
        frontLeftDrive = hwm.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hwm.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hwm.get(DcMotor.class, "front_right_drive");
        backRightDrive = hwm.get(DcMotor.class, "back_right_drive");
        lift = hwm.get(DcMotor.class, "lift");

        depositor = hwm.get(Servo.class, "depositor");
        markerHolder = hwm.get(Servo.class, "markerHolder");

        distance = hwm.get(DistanceSensor.class, "distance");

        color = hwm.get(ColorSensor.class, "color");

        //set drivetrain direction
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        //set lift direction
        lift.setDirection(DcMotor.Direction.FORWARD);

        //set init power
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        lift.setPower(0);

        //set init mode
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set init position
        depositor.setPosition(INIT);

        //setup vision if desired
        if(vision) {
            int cameraMonitorViewId = hwm.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwm.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            parameters.vuforiaLicenseKey = "AXdOcVz/////AAAAGSXgHubMS0r6roxUdjf9DWKA7GTut2LGpmgusuOdBmgcr9vnOOQAc9l3bXYlX+3nFwmePFZh1Brz4BsbF1h6zhAXHx5VvmWWpVeNoCLbxDnyaPBtmZ3k2ZgHLFLTjS2ST/arbrmCSAVUTX9xOgenNw+pcCuZYKQxr34uWWppyhJPPIPP152Gud24gReY/Sg8hM20JYH49E1nRLpIjYA1FTxWy135xOu5SCns1p48AgTLxyy51v0WRBALpIWAW/Qe30gGHb9W3swgPlBj1EVMFgLdCXgp5NAlKNvID6T6G8NaN2dYY64Cv601JDKkcGHM1KNU5N4vn6spJkE0tZI2NqfiSaBKT59+nPsbv5AGQ0IA";

            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
            this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        }
    }
    //fixes out-of-range issue with distance sensor
    public double getDistance() {
        double d = distance.getDistance(DistanceUnit.CM);
        if(isNaN(d)) d = 100.0;
        return d;
    }
    //stops chassis
    public void stop() {
        driveForward(0.0);
    }
    //power-Drive Methods
    public void driveForward(double power) {
        frontLeftDrive.setPower(power);
        backLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backRightDrive.setPower(power);
    }
    public void driveBackward(double power) { driveForward(-power);}
    public void turnLeft(double power) {
        frontLeftDrive.setPower(-power);
        backLeftDrive.setPower(-power);
        frontRightDrive.setPower(power);
        backRightDrive.setPower(power);
    }
    public void turnRight(double power) {
        turnLeft(-power);
    }
    public void strafeLeft(double power) {
        frontLeftDrive.setPower(power);
        backLeftDrive.setPower(-power);
        frontRightDrive.setPower(-power);
        backRightDrive.setPower(power);
    }
    public void strafeRight(double power) { strafeLeft(-power);}
    //drive Forward/backward to given position
    public void driveToPosition(int position) {
        warmUp();
        setTargetPosition(position);
        execute();
    }
    //set positions for all motors for drivetrain
    public void setTargetPosition(int position) {
        frontLeftDrive.setTargetPosition(position);
        backLeftDrive.setTargetPosition(position);
        frontRightDrive.setTargetPosition(position);
        backRightDrive.setTargetPosition(position);
    }
    public void setTargetTurn(int position) {
        frontLeftDrive.setTargetPosition(position);
        backLeftDrive.setTargetPosition(position);
        frontRightDrive.setTargetPosition(-position);
        backRightDrive.setTargetPosition(-position);
    }
    public void setTargetStrafe(int position) {
        frontLeftDrive.setTargetPosition(-position);
        backLeftDrive.setTargetPosition(position);
        frontRightDrive.setTargetPosition(position);
        backRightDrive.setTargetPosition(-position);
    }
    //check if motors are busy
    public boolean motorsAreBusy() {
        return (frontLeftDrive.isBusy() || backLeftDrive.isBusy() || frontRightDrive.isBusy() || backRightDrive.isBusy());
    }
    //turn to given position
    public void turnToPosition(int position) {
        warmUp();
        setTargetTurn(position);
        execute();
    }
    //encoder set-up methods for chassis
    public void warmUp() {
        stopAndReset();
        runToPosition();
    }
    public void stopAndReset() {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runToPosition() {
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void execute() {
        driveForward(0.5);
        while(motorsAreBusy()) ;
        stop();
    }
    //strafe to given position
    public void strafeToPosition(int position) {
        warmUp();
        setTargetStrafe(position);
        execute();
    }
    //lift system
    public void extendLift() {
        lift.setPower(1.0);
    }
    public void retractLift() {
        lift.setPower(-1.0);
    }

    //drive a distance method
    public void driveForwardDistance(double power, int distance){
        //reset encoders
        stopAndReset();

        //Set target position
        frontLeftDrive.setTargetPosition(distance);
        backLeftDrive.setTargetPosition(distance);
        frontRightDrive.setTargetPosition(distance);
        backRightDrive.setTargetPosition(distance);

        //Set to RUN_TO_POSITION mode
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set drive power
        driveForward(power);

        while(frontLeftDrive.isBusy() && backLeftDrive.isBusy() && frontRightDrive.isBusy() && backRightDrive.isBusy()){
            //wait until target position is reached
        }

        //stop and change modes back to normal
        stop();
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeLeftDistance(double power, int distance) {
        //reset encoders
        stopAndReset();

        //Set target position
        frontLeftDrive.setTargetPosition(distance);
        backLeftDrive.setTargetPosition(-distance);
        frontRightDrive.setTargetPosition(-distance);
        backRightDrive.setTargetPosition(distance);

        //Set to RUN_TO_POSITION mode
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set drive power
        driveForward(power);

        while(frontLeftDrive.isBusy() && backLeftDrive.isBusy() && frontRightDrive.isBusy() && backRightDrive.isBusy()){
            //wait until target position is reached
        }

        //stop and change modes back to normal
        stop();
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeRightDistance(double power, int distance){
        //reset encoders
        stopAndReset();

        //Set target position
        frontLeftDrive.setTargetPosition(-distance);
        backLeftDrive.setTargetPosition(distance);
        frontRightDrive.setTargetPosition(distance);
        backRightDrive.setTargetPosition(-distance);

        //Set to RUN_TO_POSITION mode
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set drive power
        driveForward(power);

        while(frontLeftDrive.isBusy() && backLeftDrive.isBusy() && frontRightDrive.isBusy() && backRightDrive.isBusy()){
            //wait until target position is reached
        }

        //stop and change modes back to normal
        stop();
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void findColor (){

    }
}