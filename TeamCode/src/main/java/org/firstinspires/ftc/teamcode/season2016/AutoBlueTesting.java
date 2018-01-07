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
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 This is the program for the autonomous if we are placed on the blue alliance. The objective of this
 code is to push the two beacons so they become blue and launch the balls into the center vortex.
 */

@Autonomous(name="AutoBlueTesting", group="Linear Opmode")// @Autonomous(...) is the other common choice
@Disabled
public class AutoBlueTesting extends TestRobotClass {

    public void runOpMode() throws InterruptedException {
        initializeRobot();
        //Set the servo start positions
        rightPusher.setPosition(.750);
        leftPusher.setPosition(.210);
        ballStopper.setPosition(.50);

        //Calibrate the gyro

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        gyro.calibrate();
        while(gyro.isCalibrating() && (opModeIsActive()))
        {
         //  Thread.sleep(100);
            idle();
        }
        Thread.sleep(1000);
        gyro.resetZAxisIntegrator();

        telemetry.addData("Status", "Gyro Ready");
        telemetry.update();

        double change;
        int currentGyro;
        runtime.reset();
        moveStraight(.4,100,0);
        turnDegrees(0.6, -10);
        moveStraight(0.6, 800, -45);

        /* Moves the robot forward in the direction -45 degrees relative to the start until it hits
        the wall.
         */
        leftMotor.setPower(0.2);
        rightMotor.setPower(0.2);
        while (opModeIsActive() && !(touchSensorRight.isPressed())) {
            currentGyro = gyro.getIntegratedZValue();
            change = (currentGyro - -45)/40.0;
            leftMotor.setPower(Math.max(.2 - change,0));
            rightMotor.setPower(Math.max(.2 + change,0));
            idle();
        }
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

        moveDistance(-0.4,80);
        turnDegrees(0.6, 16); //changed from 32 to 20 to 16
        //moveStraightWithAccel(.3,200,0);
        stopAtLine(0.2, 0);
        moveStraight(.2,100,0);
        pushColor();
        moveStraight(.5,700,0);
        stopAtLine(.2,0);
        moveStraight(.2,50,0);

        pushColor();

      //   Launches the balls into the center vortex
//        moveStraightWithAccel(.4,300,45);
//        moveStraightWithAccel(.4,300,180);
//        launcher.setPower(-.67);
//        sleep(1000);
//        ballStopper.setPosition(.1);
//        sleep(300);
//        ballStopper.setPosition(.50);
//        sleep(400);
//        ballStopper.setPosition(.2);
//        sleep(200);
//        ballStopper.setPosition(.50);
//        sleep(1000);
//        launcher.setPower(0);

    }

}
