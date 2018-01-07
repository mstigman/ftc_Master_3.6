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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous(name="AutoWithMethods", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class AutoWithMethods extends LinearOpMode {

    ColorSensor colorRight;
    ColorSensor colorLeft;
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor intake;
    DcMotor buttonPusher;
    OpticalDistanceSensor lineRight;
    OpticalDistanceSensor lineLeft;

    static double CountsPerRev = 1120;
    static double GearRatio = 2; //Geared up would be < 1
    static double WheelDiameter = 9.5; //cm
    static double CountsPerInch = ((CountsPerRev * GearRatio) / (WheelDiameter * 3.14159));
    int leftMotorPosition = 0;
    int rightMotorPosition = 0;
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("left motor");
        rightMotor = hardwareMap.dcMotor.get("right motor");
        buttonPusher = hardwareMap.dcMotor.get("button pusher");
        colorRight = hardwareMap.colorSensor.get("color right");
        colorLeft = hardwareMap.colorSensor.get("color left");
        intake = hardwareMap.dcMotor.get("intake");
        lineLeft = hardwareMap.opticalDistanceSensor.get("optic left");
        lineRight = hardwareMap.opticalDistanceSensor.get("optic right");
        colorRight.enableLed(false);
        colorLeft.enableLed(false);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle(); //Giving enough time to sent the information to the controller.
        idle(); //Mote time to ensure that all the data is sent to the controller.
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
waitForStart();


        while (opModeIsActive()){
            //telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Color", colorRight.blue());
            telemetry.update();
            idle();
            pushColor("blue");

        }


   }
    public void move(double speed, double distance,double endSpeed) throws InterruptedException {
        int pos = rightMotor.getCurrentPosition();
        double ticks;
        ticks = ((1120*2.5)/62.5)*distance;
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();
        if (distance > 0) {
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);

            while (leftMotor.getCurrentPosition() < (pos + ticks)) {
                idle();
            }
            leftMotor.setPower(endSpeed);
            rightMotor.setPower(endSpeed);
        }
        else
        {
            leftMotor.setPower(-speed);
            rightMotor.setPower(-speed);
            while (leftMotor.getCurrentPosition() > (pos + ticks)) {
                idle();
            }
            leftMotor.setPower(-endSpeed);
            rightMotor.setPower(-endSpeed);
        }

    }
    public void turn(double speed, int degrees) throws InterruptedException {
        int pos = rightMotor.getCurrentPosition();
        double ticks = (degrees/.42)*2;//times a constant
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();

        if (degrees > 0) {
            leftMotor.setPower(speed);
            rightMotor.setPower(-speed);
            while (leftMotor.getCurrentPosition() < (pos + ticks)) {
                idle();
            }
        }
        else
        {
            leftMotor.setPower(-speed);
            rightMotor.setPower(speed);
            while (leftMotor.getCurrentPosition() > (pos + ticks)) {
                idle();
            }
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    } public void move(double speed, double distance) throws InterruptedException {
        int pos = rightMotor.getCurrentPosition();
        double ticks;
        ticks = ((1120*2.5)/62.5)*distance;
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();
        if (distance > 0) {
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);

            while (leftMotor.getCurrentPosition() < (pos + ticks)) {
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

    public void pushColor(String Color)
    {
        colorRight.enableLed(false);
        colorLeft.enableLed(false);
        //remeber to add logic for push distance

        if (Color.equalsIgnoreCase("red"))
        {
            if (colorRight.red() >= colorRight.blue())
            {
                while (gamepad1.right_bumper)
                {
                    buttonPusher.setPower(-.1);
                }
                buttonPusher.setPower(0);
            }
            else
            {

                while (gamepad1.right_bumper)
                {
                    buttonPusher.setPower(.1);
                }
                buttonPusher.setPower(0);
            }
        }
        if (Color.equalsIgnoreCase("blue"))
        {
            if (colorLeft.red() < colorLeft.blue())
            {

                while (gamepad1.left_bumper)
                {
                    buttonPusher.setPower(.1);
                }
                buttonPusher.setPower(0);
            } else
            {
                while (gamepad1.left_bumper)
                {
                    buttonPusher.setPower(-.1);
                }
                buttonPusher.setPower(0);
            }

        }
    }
    public void allignLine(double speed) {
        double timeLeft = 100, timeRight = -100;
        int counterLeft = 1, counterRight = 1;
        boolean stateleft = true;
        boolean stateright = true;
        timer.reset();
        rightMotor.setPower(speed);
        leftMotor.setPower(speed);
        while (Math.abs((timeLeft - timeRight)) > 5)
        {
            if (lineRight.getRawLightDetected() > .3) {

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
            if (lineLeft.getRawLightDetected() > .3) {
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
    }
}
