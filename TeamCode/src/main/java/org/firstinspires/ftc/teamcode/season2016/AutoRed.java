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

@Autonomous(name="auto red", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class AutoRed extends TestRobotClass{

    @Override
    public void runOpMode() throws InterruptedException {

        rightPusher.setPosition(.702);
        leftPusher.setPosition(.225);
        ballStopper.setPosition(.50);


        // Wait for the game to start (driver presses PLAY)
        gyro.calibrate();
        while(gyro.isCalibrating() && (opModeIsActive()))
        {
            Thread.sleep(100);
            idle();
        }

        Thread.sleep(3000);
        telemetry.addData("Status", "Gyro Ready");
        telemetry.update();
        waitForStart();

        gyro.resetZAxisIntegrator();


        double change;
        int currentGyro;
        runtime.reset();
        moveStraight(.3,40,0);
        turnDegrees(0.5, 30);
        moveStraight(0.8, 1200, 45);
        leftMotor.setPower(0.15);
        rightMotor.setPower(0.15);
        double time = runtime.milliseconds();
        while (opModeIsActive() && !(touchSensorLeft.isPressed()) && (time +4000 > runtime.milliseconds())) {
            currentGyro = gyro.getIntegratedZValue();
            change = (currentGyro - 45)/40.0;
            leftMotor.setPower(Math.max(.15 - change,0));
            rightMotor.setPower(Math.max(.15 + change,0));
            idle();
        }
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

        moveDistance(-0.25,75);
        turnDegrees(0.50, -22);
        stopAtLine(0.2, 0);
        moveStraight(.2,80,0);
        pushColorRed();
        moveStraight(.6,700,0);
        stopAtLine(.2,0);
        //  moveStraightWithAccel(.1,20,0);
        pushColorRed();
        turnDegrees(.6,20);
        moveStraight(-.6,1000,45);
        launcher.setPower(-.52);
        sleep(1000);
        ballStopper.setPosition(.2);
        sleep(300);
        ballStopper.setPosition(.45);
        sleep(250);
        ballStopper.setPosition(.2);
        sleep(600);
        ballStopper.setPosition(.45);
        sleep(100);
        launcher.setPower(0);
        moveStraight(-.5,650,45);


    }

}
