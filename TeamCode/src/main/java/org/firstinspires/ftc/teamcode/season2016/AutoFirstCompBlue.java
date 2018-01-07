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
package org.firstinspires.ftc.teamcode.season2016;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoFirstCompBlue", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class AutoFirstCompBlue extends LinearOpMode {

    ColorSensor color;
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor intake;
    DcMotor buttonPusher;
    OpticalDistanceSensor lineRight;
    OpticalDistanceSensor lineLeft;
    ModernRoboticsI2cGyro gyro;
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("left motor");
        rightMotor = hardwareMap.dcMotor.get("right motor");
        buttonPusher = hardwareMap.dcMotor.get("button pusher");
        color = hardwareMap.colorSensor.get("color");
        intake = hardwareMap.dcMotor.get("intake");
        lineLeft = hardwareMap.opticalDistanceSensor.get("optic left");
        lineRight = hardwareMap.opticalDistanceSensor.get("optic right");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        color.enableLed(false);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        buttonPusher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle(); //Giving enough time to sent the information to the controller.
        idle(); //More time to ensure that all the data is sent to the controller.
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();
        gyro.calibrate();
        while(gyro.isCalibrating() && opModeIsActive())
        {
            Thread.sleep(50);
            idle();
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();



        idle();
//        runtime.reset();
//        while (opModeIsActive() && runtime.seconds() < 2){
//            telemetry.addData("value, gyro", gyro.getHeading());
//            telemetry.update();
//            idle();
//        }
        move(0.50,33,0.25);
        move(0.25, 13,.1);
        leftMotor.setPower(.1);
        rightMotor.setPower(.1);
        timer.reset();
        while (timer.milliseconds() < 1000)
        {
            idle();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sleep(100);
        //move(1,-2);
        rightMotor.setPower(-.25);
        sleep(120);
        turn(0.25, -20);
        idle();
        idle();
        sleep(200);
        allignLine(0.15);
        sleep(300);
        forwardPastLine();
        ChrispushColor();
        move(.15,7);
        forwardPastLine();
        ChrispushColor();
    }


    private void ChrispushColor() throws InterruptedException {
        int pos = buttonPusher.getCurrentPosition();
        int distance = 680;
        buttonPusher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();
        color.enableLed(false);
        //remeber to add logic for push distance
//        if (Color.equalsIgnoreCase("blue"))
//        {
            if (color.red() >= 50)
            {
                buttonPusher.setPower(-0.2);
                while (buttonPusher.getCurrentPosition() >= (pos - distance)) {
//                    telemetry.addData("red>blue", "Hello", colorLeft.red(), colorLeft.blue());
//                    telemetry.update();
                    idle();
                }
                buttonPusher.setPower(0.2);
                while (buttonPusher.getCurrentPosition() < (pos)) {
//                    telemetry.addData("red>blue", "Hello", colorLeft.red(), colorLeft.blue());
//                    telemetry.update()
                    idle();
                }
                buttonPusher.setPower(0.0);
            }
            else
            {
                buttonPusher.setPower(0.2);
                while (buttonPusher.getCurrentPosition() <= (pos + distance)) {
//                    telemetry.addData("red<blue", "Hello", colorLeft.red(), colorLeft.blue());
//                    telemetry.update();
                    idle();
                }
                buttonPusher.setPower(-0.2);
                while (buttonPusher.getCurrentPosition() > (pos)) {
//                    telemetry.addData("red>blue", "Hello", colorLeft.red(), colorLeft.blue());
//                    telemetry.update();
                    idle();
                }
                buttonPusher.setPower(0.0);
            }
//       }
//        else
//        {
//            if(colorLeft.red() >= colorLeft.blue()){
//                buttonPusher.setPower(0.2);
//            while (buttonPusher.getCurrentPosition() <= (pos + distance)) {
////                    telemetry.addData("red>blue", "Hello", colorLeft.red(), colorLeft.blue());
////                    telemetry.update();
//                idle();
//            }
//            buttonPusher.setPower(-0.2);
//            while (buttonPusher.getCurrentPosition() > (pos+8)) {
////                    telemetry.addData("red>blue", "Hello", colorLeft.red(), colorLeft.blue());
////                    telemetry.update();
//                idle();
//            }
//            buttonPusher.setPower(0.0);
//        }
//        else
//        {
//            buttonPusher.setPower(-0.2);
//            while (buttonPusher.getCurrentPosition() >= (pos - distance)) {
////                    telemetry.addData("red<blue", "Hello", colorLeft.red(), colorLeft.blue());
////                    telemetry.update();
//                idle();
//            }
//            buttonPusher.setPower(0.2);
//            while (buttonPusher.getCurrentPosition() < (pos-8)) {
////                    telemetry.addData("red>blue", "Hello", colorLeft.red(), colorLeft.blue());
////                    telemetry.update();
//                idle();
//            }
//            buttonPusher.setPower(0.0);
//        }
//    }
    }

    private void move(double speed, double distance,double endSpeed) throws InterruptedException {
        int pos = leftMotor.getCurrentPosition();
        double ticks;
        ticks = ((1120*2.5)/62.5)*distance;
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();
        if (distance > 0) {
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);

            while (opModeIsActive() && (leftMotor.getCurrentPosition() < (pos + (int)ticks))) {
                idle();
            }
            leftMotor.setPower(endSpeed);
            rightMotor.setPower(endSpeed);
        }
        else
        {
            leftMotor.setPower(-speed);
            rightMotor.setPower(-speed);
            while (opModeIsActive() && (leftMotor.getCurrentPosition() > (pos + ticks))) {
                idle();
            }
            leftMotor.setPower(-endSpeed);
            rightMotor.setPower(-endSpeed);
        }

    }
    private void turn(double speed, int degrees) throws InterruptedException {
        int start = gyro.getHeading();

        int ticks;
        ticks = start+degrees;
        if(ticks >= 360 )
        {
            ticks = ticks-360;
        }
        else if(ticks <= 0)
        {
            ticks = Math.abs(ticks);
        }

      //times a constant
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();

        if (degrees > 0) {
            //degrees = 360 - degrees;
            leftMotor.setPower(-speed);
            rightMotor.setPower(speed);
            while (opModeIsActive() && gyro.getHeading() < ticks) {
                idle();
            }
        }
        else
        {
            leftMotor.setPower(speed);
            rightMotor.setPower(-speed);
            while ((opModeIsActive() && Math.abs(gyro.getHeading()-360) < ticks) || (gyro.getHeading() < 15 && gyro.getHeading() >= 0)){

                idle();
            }
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    private void move(double speed, double distance) throws InterruptedException {
        int pos = leftMotor.getCurrentPosition();
        double ticks;
        ticks = ((1120*2.5)/62.5)*distance;
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();
        if (distance > 0) {
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);

            while (leftMotor.getCurrentPosition() < (pos + (int)ticks)) {
                idle();
            }
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
        else
        {
            leftMotor.setPower(-speed);
            rightMotor.setPower(-speed);
            while (leftMotor.getCurrentPosition() > (pos + ticks)) {
                idle();
            }
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }

    }

    private void forwardPastLine() throws InterruptedException {
        while (opModeIsActive()) {
            timer.reset();
            if (lineLeft.getLightDetected() > 0.5) {
                rightMotor.setPower(0.0);
                leftMotor.setPower(0.0);
                idle();
                break;
            } else {
                leftMotor.setPower(0.15);
                rightMotor.setPower(0.15*1.11);
            }
        }
        //move(0.15, 1.0);
    }

    private void allignLine(double speed) throws InterruptedException {
        double timeLeft = 100, timeRight = -100;
        int counterLeft = 1, counterRight = 1;
        boolean stateleft = true;
        boolean stateright = true;
        timer.reset();
        rightMotor.setPower(speed);
        leftMotor.setPower(speed);
        while (opModeIsActive() && Math.abs((timeLeft - timeRight)) > 8 )//|| (counterRight%2 == 0 && counterLeft%2 ==0))

            {
            if (lineRight.getLightDetected() > .5) {

                if (stateright) {
                    counterRight++;
                    stateright = false;
                    timeRight = timer.milliseconds();
                }

            } else {
                stateright = true;
                if (counterRight % 2 == 0) {
                    rightMotor.setPower(-speed);

                }
            else {
                    rightMotor.setPower(speed);
                }
            }
            if (lineLeft.getLightDetected() > .5) {
                if (stateleft) {
                    counterLeft++;
                    stateleft = false;
                    timeLeft = timer.milliseconds();
                }
            } else {
                stateleft = true;
                if (counterLeft % 2 == 0) {
                    leftMotor.setPower(-speed);
                }
                else {
                    leftMotor.setPower(speed);
                }
            }
        }
        double back = .5;
        if(counterRight%2 == 0)
        {
            back = 1;
        }
        leftMotor.setPower(-0.1);
        rightMotor.setPower(-0.1);
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < back){
            idle();
        }
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }
}
