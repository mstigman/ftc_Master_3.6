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

/*
 This is the program for the autonomous if we are placed on the blue alliance. The objective of this
 code is to push the two beacons so they become blue and launch the balls into the center vortex.
 */

@Autonomous(name="auto blue with block", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class AutoBlueWithBlock extends TestRobotClass {
    @Override
    public void runOpMode() throws InterruptedException {

        initializeRobot();

        rightPusher.setPosition(.750);
        leftPusher.setPosition(.210);
        ballStopper.setPosition(.50);

        //Calibrate the gyro

        // Wait for the game to start (driver presses PLAY)
        gyro.calibrate();
        while(gyro.isCalibrating() && (opModeIsActive()))
        {
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
        moveStraight(.4,45,0);
        turnDegrees(0.5, -30);
        moveStraight(0.8,1250, -45);
        //Thread.sleep(7000);
        /* Moves the robot forward in the direction -45 degrees relative to the start until it hits
        the wall.
         */
        leftMotor.setPower(0.15);
        rightMotor.setPower(0.15);
        while (opModeIsActive() && !(touchSensorRight.isPressed()) && (time +4000 > runtime.milliseconds())) {
            currentGyro = gyro.getIntegratedZValue();
            change = (currentGyro - -45)/40.0;
            leftMotor.setPower(Math.max(.15 - change,0));
            rightMotor.setPower(Math.max(.15 + change,0));
            idle();
        }
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

        moveDistance(-0.3,80);
        turnDegrees(0.6, 16);
        stopAtLine(0.2, 0);
        moveStraight(.2,80,0);
        pushColor();
        moveStraight(.6,700,0);
        stopAtLine(.2,0);
      //  moveStraightWithAccel(.1,20,0);
        pushColor();
        turnDegrees(.6,-15);
        moveStraight(-.6,1050,-44);
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
        turnDegrees(.5,30);
        moveStraight(.8,1000,25);

    }

}
