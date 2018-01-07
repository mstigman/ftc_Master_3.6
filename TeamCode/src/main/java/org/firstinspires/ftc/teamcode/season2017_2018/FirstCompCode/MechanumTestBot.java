/**
 * Created by FTC on 3/19/2017.
 */
package org.firstinspires.ftc.teamcode.season2017_2018.FirstCompCode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;

public abstract class MechanumTestBot extends LinearOpMode{

    private final int accelAverageSize = 100;
    protected  int armUp = -100;
    protected final double ballKnockUp = .3;
    protected final double ballKnockDown = .8;
    protected final int armDown = 800;
    protected final int bucketDown = 0;
    protected final int bucketUp  = 575;
    protected final double rightGrabbed = .72;
    protected final double rightOff = .45;
    protected final double leftGrabbed = .28;
    protected final double leftOff = .55;
    protected final double bucketSpeed = .6;
    protected final double armSpeed = .6;
    private final int delayForBlocks = 250;
    protected ElapsedTime runtime = new ElapsedTime();
    protected Servo rightGrab = null;
    protected Servo leftGrab = null;
    protected Servo ballKnocker = null;
    protected DcMotor leftUpMotor = null;
    protected DcMotor pickArm = null;
    protected  DcMotor dumpBucket = null;
    protected DcMotor rightUpMotor = null;
    protected DcMotor leftDownMotor = null;
    protected  DcMotor rightDownMotor = null;
    protected int gyroposition = 0;
    protected ModernRoboticsI2cGyro gyro;
    protected ColorSensor color;
    protected OpticalDistanceSensor ODS;
    protected ModernRoboticsI2cRangeSensor range = null;
    protected double gyroAdjust = 40.0;
    private int alignTime = 500;
    private boolean turned = false;
    private boolean turning = false;
    protected double gyroStart;
    protected VuforiaLocalizer vuforia;
    //call before tou do anything, sets up robot
    protected void initializeRobot() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AfWRMZn/////AAAAGeL3ADUJQUPGhNAhNQUth5aLwwOlSO08asw1uqJQF1ndzliHZxf3aUyFBY7KSvgH1zxdJlE+WRbhM/HwOddafzk+SEMQjJ+u6udl9ooyIAHjK6Vh3GBu5aYDYUoe2gM0/7mMvEy+4OAGlhvQ6ZYdRlBVc2FeK1jJsW9eeELZC/i5fhoSnQtWKBiD8YDQngbRTz8SHJGfWbJCCKg+c6QRpNRFWmRiwpjh0hIN+GeyZLw6GR3vcRl3b6vPZEBrwK018Tq98jRY/le3z5egiVmP6+VE/Vuw6LN7GekcATsB9n6hM2ukKhWswp6ZbzTVwSW5+0PJrr78t6OCdA4r0JMVyi9OUlbz4SC+LR+4b23BQzz9";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        leftUpMotor = hardwareMap.dcMotor.get("front left");
        dumpBucket = hardwareMap.dcMotor.get("dump bucket");
        rightUpMotor = hardwareMap.dcMotor.get("front right");
        leftDownMotor = hardwareMap.dcMotor.get("back left");
        pickArm = hardwareMap.dcMotor.get("arm");
        rightDownMotor = hardwareMap.dcMotor.get("back right");
        rightGrab = hardwareMap.servo.get("right grab");
        leftGrab = hardwareMap.servo.get("left grab");
        ballKnocker = hardwareMap.servo.get("ball knocker");
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        color = hardwareMap.colorSensor.get("color");
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range finder");
        //color.setI2cAddress(I2cAddr.create7bit(0x3c));
        leftUpMotor.setDirection(DcMotor.Direction.FORWARD);
        rightUpMotor.setDirection(DcMotor.Direction.REVERSE);
        leftDownMotor.setDirection(DcMotor.Direction.FORWARD);
        rightDownMotor.setDirection(DcMotor.Direction.REVERSE);
        dumpBucket.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dumpBucket.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pickArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pickArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pickArm.setDirection(DcMotor.Direction.REVERSE);
        color.enableLed(true);
        leftGrab.setPosition(leftOff);
        rightGrab.setPosition(rightOff);
        ballKnocker.setPosition(ballKnockUp);
        resetGyro();
    }


    //moves the robot in a specified direction at a specified power for a specified distance. uses gyro
    //dont get hung up on the distance 1000 forward may not be the same as 1000 to the right (though it should be)
    //it is constant though
    protected void moveStraightOld(double motorPower, int distance, double direction)throws InterruptedException
    {
        double yComp = Math.sin(Math.toRadians(direction));
        double xComp = Math.cos(Math.toRadians(direction));
        rightUpMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDownMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightUpMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDownMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentGyro;
        double change;
        int rightUpSwitch;
        int rightDownSwitch;
        double upRightPower = yComp - xComp;
        double upLeftPower = yComp + xComp;
        double downRightPower = yComp + xComp;
        double downLeftPower = yComp - xComp;
        double largest = 0;
        int goalTicks;
        if(Math.abs(upRightPower) > largest)
        {
            largest = Math.abs(upRightPower);
        }
        if(Math.abs(upLeftPower) > largest)
        {
            largest = Math.abs(upLeftPower);
        }
        if(Math.abs(downRightPower) > largest)
        {
            largest = Math.abs(downRightPower);
        }
        if(Math.abs(downLeftPower) > largest)
        {
            largest = Math.abs(downLeftPower);
        }
        double offset = 1/largest;
        upRightPower = upRightPower*offset*motorPower;
        upLeftPower = upLeftPower*offset*motorPower;
        downRightPower = downRightPower*offset*motorPower;
        downLeftPower = downLeftPower*offset*motorPower;
        //   https://robotics.stackexchange.com/questions/65/calculating-the-efficiency-of-mecanum-wheel
        if(Math.abs(xComp) > Math.abs(yComp))
        {
            goalTicks =  Math.abs((int)(distance*Math.cos(Math.toRadians(direction))));
            while(opModeIsActive() && Math.abs((rightUpMotor.getCurrentPosition()) - (rightDownMotor.getCurrentPosition()))/2 < goalTicks)
            {
                currentGyro = gyro.getIntegratedZValue();
                change = (currentGyro - gyroStart)/gyroAdjust;
                // change from linear equation
                leftUpMotor.setPower(upLeftPower + change);
                leftDownMotor.setPower(downLeftPower + change);
                rightUpMotor.setPower(upRightPower - change);
                rightDownMotor.setPower(downRightPower - change);
                idle();
            }
        }
        else
        {
            goalTicks =  Math.abs((int)(distance*Math.sin(Math.toRadians(direction))));
            while(opModeIsActive() && Math.abs((rightUpMotor.getCurrentPosition()) + (rightDownMotor.getCurrentPosition()))/2 < goalTicks)
            {
                currentGyro = gyro.getIntegratedZValue();
                //same
                change = (currentGyro - gyroStart)/gyroAdjust;
                leftUpMotor.setPower(upLeftPower + change);
                leftDownMotor.setPower(downLeftPower + change);
                rightUpMotor.setPower(upRightPower - change);
                rightDownMotor.setPower(downRightPower - change);
                idle();
            }
        }
        leftUpMotor.setPower(0);
        leftDownMotor.setPower(0);
        rightUpMotor.setPower(0);
        rightDownMotor.setPower(0);
        align();

    }
    //same as other but no direction, useful in loops
    protected void moveStraightOld(double motorPower, double direction)throws InterruptedException
    {
        double yComp = Math.sin(Math.toRadians(direction));
        double xComp = Math.cos(Math.toRadians(direction));
        int currentGyro;
        double change;
        int rightUpSwitch;
        int rightDownSwitch;
        double upRightPower = yComp - xComp;
        double upLeftPower = yComp + xComp;
        double downRightPower = yComp + xComp;
        double downLeftPower = yComp - xComp;
        double largest = 0;
        if(Math.abs(upRightPower) > largest)
        {
            largest = Math.abs(upRightPower);
        }
        if(Math.abs(upLeftPower) > largest)
        {
            largest = Math.abs(upLeftPower);
        }
        if(Math.abs(downRightPower) > largest)
        {
            largest = Math.abs(downRightPower);
        }
        if(Math.abs(downLeftPower) > largest)
        {
            largest = Math.abs(downLeftPower);
        }
        double offset = 1/largest;
        upRightPower = upRightPower*offset*motorPower;
        upLeftPower = upLeftPower*offset*motorPower;
        downRightPower = downRightPower*offset*motorPower;
        downLeftPower = downLeftPower*offset*motorPower;
        if(Math.abs(xComp) > Math.abs(yComp))
        {

            currentGyro = gyro.getIntegratedZValue();
            change = (currentGyro - gyroStart)/gyroAdjust;
            leftUpMotor.setPower(upLeftPower + change);
            leftDownMotor.setPower(downLeftPower + change);
            rightUpMotor.setPower(upRightPower - change);
            rightDownMotor.setPower(downRightPower - change);
            idle();

        }
        else
        {

            currentGyro = gyro.getIntegratedZValue();
            change = (currentGyro - gyroStart)/gyroAdjust;
            leftUpMotor.setPower(upLeftPower + change);
            leftDownMotor.setPower(downLeftPower + change);
            rightUpMotor.setPower(upRightPower - change);
            rightDownMotor.setPower(downRightPower - change);
            idle();

        }
    }

    // Moves the robot the given distance in encoders at the given speed.
    //doesnt use the gyro
    protected void move(double motorPower, int encoderTicks, double direction)throws InterruptedException {
        double yComp = Math.sin(Math.toRadians(direction));
        double xComp = Math.cos(Math.toRadians(direction));
        rightUpMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDownMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightUpMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDownMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double upRightPower = yComp - xComp;
        double upLeftPower = yComp + xComp;
        double downRightPower = yComp + xComp;
        double downLeftPower = yComp - xComp;
        double largest = 0;
        int goalTicks;
        int ticksDone = 0;
        if(Math.abs(upRightPower) > largest)
        {
            largest = Math.abs(upRightPower);
        }
        if(Math.abs(upLeftPower) > largest)
        {
            largest = Math.abs(upLeftPower);
        }
        if(Math.abs(downRightPower) > largest)
        {
            largest = Math.abs(downRightPower);
        }
        if(Math.abs(downLeftPower) > largest)
        {
            largest = Math.abs(downLeftPower);
        }
        double offset = 1/largest;
        upRightPower = upRightPower*offset*motorPower;
        upLeftPower = upLeftPower*offset*motorPower;
        downRightPower = downRightPower*offset*motorPower;
        downLeftPower = downLeftPower*offset*motorPower;
        //   https://robotics.stackexchange.com/questions/65/calculating-the-efficiency-of-mecanum-wheel
        if(Math.abs(xComp) > Math.abs(yComp))
        {
            goalTicks =  Math.abs((int)(encoderTicks*Math.cos(Math.toRadians(direction))));
            while(opModeIsActive() && ticksDone < goalTicks)
            {
                ticksDone = Math.abs((rightUpMotor.getCurrentPosition()) - (rightDownMotor.getCurrentPosition()))/2;
                //showEncoders();
                if(goalTicks - ticksDone > 100)
                {
                    leftUpMotor.setPower(upLeftPower);
                    leftDownMotor.setPower(downLeftPower);
                    rightUpMotor.setPower(upRightPower);
                    rightDownMotor.setPower(downRightPower);
                }
                else
                {
                    //  stopMotors(100,ticksDone,upRightPower,upLeftPower,downRightPower,downLeftPower);
                }
                idle();
            }
        }
        else
        {
            goalTicks =  Math.abs((int)(encoderTicks*Math.sin(Math.toRadians(direction))));
            while(opModeIsActive() && ticksDone < goalTicks)
            {
                ticksDone = Math.abs((rightUpMotor.getCurrentPosition()) + (rightDownMotor.getCurrentPosition()))/2;
                //showEncoders();
                if(goalTicks - ticksDone > 100)
                {
                    leftUpMotor.setPower(upLeftPower);
                    leftDownMotor.setPower(downLeftPower);
                    rightUpMotor.setPower(upRightPower);
                    rightDownMotor.setPower(downRightPower);
                }
                else
                {
                    //   stopMotors(100,ticksDone,upRightPower,upLeftPower,downRightPower,downLeftPower);
                }
                idle();
            }
        }
        leftUpMotor.setPower(0);
        leftDownMotor.setPower(0);
        rightUpMotor.setPower(0);
        rightDownMotor.setPower(0);


    }
    // no gyro takes components as opposed to power and direction
    protected void moveDrive(double xComp, double yComp, double turn)throws InterruptedException {
        double motorPower = Math.sqrt(Math.pow(xComp,2)+Math.pow(yComp,2) + Math.pow(turn,2));
        if(motorPower > 1)
        {
            motorPower = 1;
        }
        if (motorPower < -1)
        {
            motorPower = -1;
        }
        double upRightPower = yComp - xComp - turn;
        double upLeftPower = yComp + xComp + turn;
        double downRightPower = yComp + xComp - turn;
        double downLeftPower = yComp - xComp + turn;
        double largest = 0;
        if(Math.abs(upRightPower) > largest)
        {
            largest = Math.abs(upRightPower);
        }
        if(Math.abs(upLeftPower) > largest)
        {
            largest = Math.abs(upLeftPower);
        }
        if(Math.abs(downRightPower) > largest)
        {
            largest = Math.abs(downRightPower);
        }
        if(Math.abs(downLeftPower) > largest)
        {
            largest = Math.abs(downLeftPower);
        }
        double offset = 1;
        if(largest != 0)
        {
            offset = 1/largest;
        }
        upRightPower = upRightPower*offset*motorPower;
        upLeftPower = upLeftPower*offset*motorPower;
        downRightPower = downRightPower*offset*motorPower;
        downLeftPower = downLeftPower*offset*motorPower;
        if(xComp == 0 && yComp == 0 && turn == 0)
        {
            leftUpMotor.setPower(0);
            leftDownMotor.setPower(0);
            rightUpMotor.setPower(0);
            rightDownMotor.setPower(0);
            return;
        }
        telemetry.update();
        leftUpMotor.setPower(upLeftPower);
        leftDownMotor.setPower(downLeftPower);
        rightUpMotor.setPower(upRightPower);
        rightDownMotor.setPower(downRightPower);
    }
    //uses gyro takes components
    //turn is -1 to 1 its just how powerful the turn is
    protected void moveStraightDrive(double xComp, double yComp, double turn)throws InterruptedException {
        double motorPower = Math.sqrt(Math.pow(xComp,2)+Math.pow(yComp,2));
        if(motorPower > .8)
        {
            motorPower = .8;
        }
        if (motorPower < -.8)
        {
            motorPower = -.8;
        }
        double upRightPower = yComp - xComp;
        double upLeftPower = yComp + xComp;
        double downRightPower = yComp + xComp;
        double downLeftPower = yComp - xComp;
        int currentGyro;
        double change;
        double turnTime = 0;
        if(turn > .25 || turn < -.25)
        {
            gyroStart = gyro.getIntegratedZValue() - Math.min((turn*150),150);
            turning = true;
        }
        else
        {
            if(Math.abs(gyroStart - gyro.getIntegratedZValue()) > 20)
            {
                gyroStart = gyro.getIntegratedZValue();
            }

        }
        double largest = 0;
        if(Math.abs(upRightPower) > largest)
        {
            largest = Math.abs(upRightPower);
        }
        if(Math.abs(upLeftPower) > largest)
        {
            largest = Math.abs(upLeftPower);
        }
        if(Math.abs(downRightPower) > largest)
        {
            largest = Math.abs(downRightPower);
        }
        if(Math.abs(downLeftPower) > largest)
        {
            largest = Math.abs(downLeftPower);
        }
        double offset = 1;
        if(largest != 0)
        {
            offset = 1/largest;
        }
        upRightPower = upRightPower*offset*motorPower;
        upLeftPower = upLeftPower*offset*motorPower;
        downRightPower = downRightPower*offset*motorPower;
        downLeftPower = downLeftPower*offset*motorPower;
        if(xComp == 0 && yComp == 0)
        {
            upRightPower = 0;
            upLeftPower = 0;
            downRightPower = 0;
            downLeftPower = 0;
        }
        currentGyro = gyro.getIntegratedZValue();
        change = (currentGyro - gyroStart)/100.0;
        if(change < .1 && change > -.1)
        {
            change = 0;
        }
        leftUpMotor.setPower(upLeftPower + change);
        leftDownMotor.setPower(downLeftPower + change);
        rightUpMotor.setPower(upRightPower - change);
        rightDownMotor.setPower(downRightPower - change);
        idle();
    }
    //aligns the robot to correct orrientation
    private void align()
    {
        double startTime = runtime.milliseconds();
        double currentGyro;
        double changeGyro;
        while(opModeIsActive() && runtime.milliseconds() - startTime < alignTime)
        {
            currentGyro = gyro.getIntegratedZValue();
            changeGyro = (currentGyro - gyroStart)/gyroAdjust;
            leftUpMotor.setPower(+changeGyro);
            leftDownMotor.setPower(+changeGyro);
            rightUpMotor.setPower(-changeGyro);
            rightDownMotor.setPower(-changeGyro);
            idle();
        }
    }
    //  Turns the robot the given amount of degrees at the given speed.
    protected void turnDegrees (double motorPower, int gyroDegrees)throws InterruptedException
    {
        gyroStart += gyroDegrees;
        gyroposition = gyro.getIntegratedZValue();
        if (gyroDegrees > 0) {
            leftUpMotor.setPower(-motorPower);
            leftDownMotor.setPower(-motorPower);
            rightDownMotor.setPower(motorPower);
            rightUpMotor.setPower(motorPower);
            while (opModeIsActive() && (gyro.getIntegratedZValue() < (gyroposition+gyroDegrees-(gyroDegrees*.1)))) {


                idle();
            }
        }
        else {
            leftUpMotor.setPower(motorPower);
            leftDownMotor.setPower(motorPower);
            rightDownMotor.setPower(-motorPower);
            rightUpMotor.setPower(-motorPower);
            while (opModeIsActive() && (gyro.getIntegratedZValue() > (gyroposition+gyroDegrees-(gyroDegrees*.1)))) {


                idle();
            }
        }
        // run until the end of the match (driver presses STOP)
        rightDownMotor.setPower(0.0);
        rightUpMotor.setPower(0.0);
        leftUpMotor.setPower(0.0);
        rightUpMotor.setPower(0.0);
        align();

    }
    //moves forward then to the side
    //number is the hole it stops at
    protected void stopAtGlyphRed(int nubmer) throws InterruptedException
    {
        int numberAt = 0;
        boolean seen = false;
        while(range.getDistance(DistanceUnit.CM) > 33)
        {
            moveStraightOld(.2,270);
        }
        align();
        while (opModeIsActive() && numberAt < nubmer)
        {
            moveStraightOld(.5,-2);

            showRange();
            telemetry.addData("seen: ", numberAt);
            telemetry.update();
            if(range.getDistance(DistanceUnit.CM) <= 19 && !seen)
            {
                seen = true;
                numberAt++;
            }
            else
            {
                if(range.getDistance(DistanceUnit.CM) > 19)
                {
                    seen = false;
                }
            }
        }
        align();
        if(numberAt == 1)
        {
            moveStraightOld(.6,150,180);
        }
        else {
            moveStraightOld(.6, 210, 180);
        }
        align();
        moveStraightOld(0,0,0);
    }
    protected void stopAtGlyphBlue(int nubmer) throws InterruptedException
    {
        int numberAt = 0;
        boolean seen = false;
        while(range.getDistance(DistanceUnit.CM) > 33)
        {
            moveStraightOld(.2,270);
        }
        align();
        while (opModeIsActive() && numberAt < nubmer);
        {
            moveStraightOld(.5,0);

            showRange();
            telemetry.addData("seen: ", numberAt);
            telemetry.update();
            if(range.getDistance(DistanceUnit.CM) <= 19 && !seen)
            {
                seen = true;
                numberAt++;
            }
            else
            {
                if(range.getDistance(DistanceUnit.CM) > 19)
                {
                    seen = false;
                }
            }
        }
        align();
        if(numberAt == 1)
        {
            moveStraightOld(.6,150,180);
        }
        else {
            moveStraightOld(.6, 210, 180);
        }
        align();
        moveStraightOld(0,0,0);
    }
    protected int vuforiaInt() {
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTrackables.activate();
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        int VuMarkLeft = 3;
        int VuMarkCenter = 2;
        int VuMarkRight = 1;
        runtime.reset();
        while(opModeIsActive() && runtime.milliseconds() < 3000)
        {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (vuMark != RelicRecoveryVuMark.UNKNOWN)
            {
                if (vuMark == RelicRecoveryVuMark.LEFT)
                {
                    return VuMarkLeft;
                }
                if (vuMark == RelicRecoveryVuMark.CENTER)
                {
                    return VuMarkCenter;
                }
                if (vuMark == RelicRecoveryVuMark.RIGHT)
                {
                    return VuMarkRight;
                }
            }
        }
        return  0;
    }
    protected void blockDistance() throws InterruptedException {
        double change = range.getDistance(DistanceUnit.CM) * .05;
        moveStraightDrive(change, 0, 0);
    }
    protected void stopMotors()
    {
        leftDownMotor.setPower(0);
        leftUpMotor.setPower(0);
        rightDownMotor.setPower(0);
        rightUpMotor.setPower(0);
    }
    protected void resetGyro()
    {
        double time = runtime.milliseconds();
        gyro.calibrate();
        gyroStart = 0;
        while(runtime.milliseconds()-time < 3000)
        {
            idle();
        }
    }
    protected boolean isBlue(){
        ballKnocker.setPosition(ballKnockDown);
        sleep(500);
        int colorVal = color.blue();
        sleep(200);
        if(color.blue() > 71){
            return true;
        }
        else{
            return false;
        }
    }
    protected void pickUpBlock(boolean check_right_bumper)
    {
        if((gamepad1.right_bumper || !check_right_bumper))
        {
            rightGrab.setPosition(rightOff);
            leftGrab.setPosition(leftOff);
        }
        while(opModeIsActive() && (gamepad1.right_bumper || !check_right_bumper) && pickArm.getCurrentPosition() < armDown)
        {
            pickArm.setPower(armSpeed);
        }
        pickArm.setPower(0);
        sleep(delayForBlocks);
        if((gamepad1.right_bumper || !check_right_bumper))
        {
            rightGrab.setPosition(rightGrabbed);
            leftGrab.setPosition(leftGrabbed);
            sleep(delayForBlocks);
        }
        while(opModeIsActive() && (gamepad1.right_bumper || !check_right_bumper) && pickArm.getCurrentPosition() > armUp)
        {
            pickArm.setPower(-armSpeed);
        }
        pickArm.setPower(0);
        sleep(delayForBlocks);
        if((gamepad1.right_bumper || !check_right_bumper))
        {
            rightGrab.setPosition(rightOff);
            leftGrab.setPosition(leftOff);
        }
        while(opModeIsActive() && (gamepad1.right_bumper && check_right_bumper))
        {

        }

    }
    protected void dumpBlocks(boolean check_left_bumper) throws InterruptedException {
        if(gamepad1.right_bumper) {
            rightGrab.setPosition(rightOff);
            leftGrab.setPosition(leftOff);
        }
        while(opModeIsActive() && (gamepad1.left_bumper || !check_left_bumper) && pickArm.getCurrentPosition() < armDown)
        {
            pickArm.setPower(armSpeed);
        }
        pickArm.setPower(0);
        while(opModeIsActive() && (gamepad1.left_bumper || !check_left_bumper)  && dumpBucket.getCurrentPosition() < bucketUp)
        {
            dumpBucket.setPower(bucketSpeed);
        }
        dumpBucket.setPower(0);
        if(gamepad1.left_bumper)
        {
            sleep(500);
        }
        else if(!check_left_bumper)
        {
            sleep(1500);
        }
        while(opModeIsActive() && (gamepad1.left_bumper || !check_left_bumper)  && dumpBucket.getCurrentPosition() > bucketDown)
        {
            dumpBucket.setPower(-bucketSpeed);
        }
        dumpBucket.setPower(0);
        while(opModeIsActive() && (gamepad1.left_bumper || !check_left_bumper)  && pickArm.getCurrentPosition() > armUp)
        {
            pickArm.setPower(-armSpeed);
        }
        pickArm.setPower(0);
        while(opModeIsActive() && (gamepad1.left_bumper && check_left_bumper))
        {

        }
        if (!check_left_bumper)
        {
            moveStraightOld(.25,100,90);
        }

    }
    protected void tryToNotFail()
    {
        dumpBucket.setPower(1);
        sleep(150);
        setBucketDown();
    }
    protected void setBucketDown()
    {
        while(opModeIsActive() && dumpBucket.getCurrentPosition() > bucketDown)
        {
            dumpBucket.setPower(-bucketSpeed);
        }
        dumpBucket.setPower(0);
    }
    protected void setArmUp()
    {
        setBucketDown();
        while(opModeIsActive() && pickArm.getCurrentPosition() > armUp)
        {
            pickArm.setPower(-armSpeed);
        }
        pickArm.setPower(0);
    }
    protected void showEncoders()
    {
        telemetry.addData("left up encoder data: ", leftUpMotor.getCurrentPosition());
        telemetry.addData("right up encoder data: ", rightUpMotor.getCurrentPosition());
        telemetry.addData("left down encoder data: ", leftDownMotor.getCurrentPosition());
        telemetry.addData("right down encoder data: ", rightDownMotor.getCurrentPosition());
        telemetry.addData("dump bucket encoder data", dumpBucket.getCurrentPosition());
        telemetry.addData("pick arm encoder data", pickArm.getCurrentPosition());
        telemetry.addData("right grab servo", rightGrab.getPosition());
        telemetry.addData("left grab servo", leftGrab.getPosition());
    }
    protected void showGyro() {

        telemetry.addData("gyro data: ", gyro.getIntegratedZValue());
    }
    protected void showRange()
    {
        telemetry.addData("distance cm: ", range.getDistance(DistanceUnit.CM));
    }

}
