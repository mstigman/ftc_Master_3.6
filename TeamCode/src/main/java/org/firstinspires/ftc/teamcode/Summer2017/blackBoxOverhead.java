/**
 * Created by FTC on 3/19/2017.
 */
package org.firstinspires.ftc.teamcode.Summer2017;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class blackBoxOverhead extends LinearOpMode{

    protected ElapsedTime runtime = new ElapsedTime();
    protected DcMotor leftMotor = null;
    protected DcMotor rightMotor = null;
    protected DcMotor launcher;
    protected int gyroposition = 0;
    protected ModernRoboticsI2cGyro gyro;
    protected TouchSensor touchSensorRight;
    protected TouchSensor touchSensorLeft;
    protected OpticalDistanceSensor opticRight = null;
    protected OpticalDistanceSensor opticLeft = null;
    protected ColorSensor colorRight;
    protected ColorSensor colorLeft;
    protected Servo rightPusher;
    protected Servo leftPusher;
    protected Servo ballStopper;
    protected void initializeRobot()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftMotor  = hardwareMap.dcMotor.get("left drive");
        rightMotor = hardwareMap.dcMotor.get("right drive");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        touchSensorLeft = hardwareMap.touchSensor.get("touch sensor right");
        touchSensorRight = hardwareMap.touchSensor.get("touch sensor left");
        opticRight = hardwareMap.opticalDistanceSensor.get("optic left");
        opticLeft = hardwareMap.opticalDistanceSensor.get("optic right");
        colorRight = hardwareMap.colorSensor.get("color right");
        colorLeft = hardwareMap.colorSensor.get("color left");
        rightPusher = hardwareMap.servo.get("right pusher");
        leftPusher = hardwareMap.servo.get("left pusher");
        ballStopper = hardwareMap.servo.get("ball stopper");
        launcher = hardwareMap.dcMotor.get("launcher");
        colorLeft.setI2cAddress(I2cAddr.create7bit(0x1e));
        colorRight.setI2cAddress(I2cAddr.create7bit(0x26));
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    protected void pushColor() throws InterruptedException
    {
        sleep(200);
        if(colorRight.blue()>colorRight.red())
        {
            rightPusher.setPosition(.384);
            sleep(900);
            rightPusher.setPosition(.702);
        }
        else
        {
            moveStraight(.35,190,0);
            rightPusher.setPosition(.384);
            sleep(900);
            rightPusher.setPosition(.702);
        }
    }
    protected void moveStraight(double motorPower, int encoderTicks, int headingFromStart)throws InterruptedException {

        int leftPosition = 0;
        int rightPosition = 0;
        int gyroStart = headingFromStart; //gyro.getIntegratedZValue();
        encoderTicks = Math.abs(encoderTicks);
        int currentGyro;
        double change;
        leftMotor.setPower(motorPower);
        rightMotor.setPower(motorPower);

        leftPosition = leftMotor.getCurrentPosition();
        rightPosition = rightMotor.getCurrentPosition();

        if (motorPower > 0) {
            while ((leftMotor.getCurrentPosition() < (leftPosition + encoderTicks) ||
                    (rightMotor.getCurrentPosition() < (rightPosition + encoderTicks))) &&
                    opModeIsActive()){
                currentGyro = gyro.getIntegratedZValue();
                change = (currentGyro - gyroStart)/40.0;
                leftMotor.setPower(Math.max(motorPower - change,0));
                rightMotor.setPower(Math.max(motorPower + change,0));
                idle();
            }
        }
        else {

            while ((leftMotor.getCurrentPosition() > (leftPosition - encoderTicks) ||
                    (rightMotor.getCurrentPosition() > (rightPosition - encoderTicks))) &&
                    opModeIsActive()){
                currentGyro = gyro.getIntegratedZValue();
                change = (currentGyro - gyroStart)/40.0;
                leftMotor.setPower(Math.min(motorPower - change,0));
                rightMotor.setPower(Math.min(motorPower + change,0));
                idle();
            }
        }


        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

    }
    // Moves the robot the given distance in encoders at the given speed.
    protected void moveDistance(double motorPower, int encoderTicks)throws InterruptedException {

        int leftPosition = 0;
        int rightPosition = 0;

        encoderTicks = Math.abs(encoderTicks);

        leftMotor.setPower(motorPower);
        rightMotor.setPower(motorPower);

        leftPosition = leftMotor.getCurrentPosition();
        rightPosition = rightMotor.getCurrentPosition();

        if (motorPower > 0) {
            while (leftMotor.getCurrentPosition() < (leftPosition + encoderTicks) &&
                    (rightMotor.getCurrentPosition() < (rightPosition + encoderTicks)) &&
                    opModeIsActive()){

                idle();
            }
        }
        else {

            while (leftMotor.getCurrentPosition() > (leftPosition - encoderTicks) &&
                    (rightMotor.getCurrentPosition() > (rightPosition - encoderTicks)) &&
                    opModeIsActive()){

                idle();        }


            leftMotor.setPower(0.0);
            rightMotor.setPower(0.0);

        }
    }
    //  Turns the robot the given amount of degrees at the given speed.
    protected void turnDegrees (double motorPower, int gyroDegrees)throws InterruptedException {

        gyroposition = gyro.getIntegratedZValue();
        if (gyroDegrees > 0) {
            leftMotor.setPower(motorPower);
            rightMotor.setPower(-motorPower);
            while (opModeIsActive() && (gyro.getIntegratedZValue()<(gyroposition+gyroDegrees))) {


                idle();
            }
        }
        else {
            leftMotor.setPower(-motorPower);
            rightMotor.setPower(motorPower);
            while (opModeIsActive() && (gyro.getIntegratedZValue()>(gyroposition+gyroDegrees))) {


                idle();
            }
        }





        // run until the end of the match (driver presses STOP)

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

    }
    // Moves the robot forward until it get to the white line, and then stops.
    protected void stopAtLine (double motorPower, int headingFromStart) throws InterruptedException {

        int gyroStart = headingFromStart;
        int currentGyro;
        double change;

        leftMotor.setPower(motorPower);
        rightMotor.setPower(motorPower);

        while (opModeIsActive() && (opticLeft.getLightDetected() < 0.1 && opticRight.getLightDetected() < 0.1)) {

            currentGyro = gyro.getIntegratedZValue();
            change = (currentGyro - gyroStart)/40.0;
            leftMotor.setPower(Math.max(motorPower - change,0));
            rightMotor.setPower(Math.max(motorPower + change,0));
            idle();
        }

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }
    protected void pushColorRed() throws InterruptedException
    {
        sleep(200);
        if(colorLeft.blue()>colorLeft.red())
        {
            moveStraight(.35,200,0);
            leftPusher.setPosition(.548);
            sleep(800);
            leftPusher.setPosition(.225);
        }
        else
        {
            leftPusher.setPosition(.548);
            sleep(800);
            leftPusher.setPosition(.225);
        }
    }
}
