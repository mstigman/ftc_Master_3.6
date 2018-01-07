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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 This is the program for the autonomous if we are placed on the blue alliance. The objective of this
 code is to push the two beacons so they become blue and launch the balls into the center vortex.
 */

@Autonomous(name="cap ball red", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class CapBallRed extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor launcher;
    int gyroposition = 0;
    ModernRoboticsI2cGyro gyro;
    TouchSensor touchSensorRight;
    TouchSensor touchSensorLeft;
    OpticalDistanceSensor opticRight = null;
    OpticalDistanceSensor opticLeft = null;
    ColorSensor colorRight;
    ColorSensor colorLeft;
    Servo rightPusher;
    Servo leftPusher;
    Servo ballStopper;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
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

        // Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        //Set the servo start positions
        rightPusher.setPosition(.702);
        leftPusher.setPosition(.225);
        ballStopper.setPosition(.50);

       // Wait for the game to start (driver presses PLAY)
        waitForStart();

        sleep(15000);
        moveDistance(1, 3000);
        rightMotor.setPower(.5);
        leftMotor.setPower(0.0);
        sleep(200);
        moveDistance(.6,2500);

    }

    // Moves the robot the given distance in encoders at the given speed.
    private void moveDistance(double motorPower, int encoderTicks)throws InterruptedException {

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
}
