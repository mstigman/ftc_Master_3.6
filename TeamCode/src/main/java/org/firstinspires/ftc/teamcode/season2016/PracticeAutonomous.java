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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
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

@TeleOp(name="PracticeAutonomous")  // @Autonomous(...) is the other common choice
@Disabled
public class PracticeAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    int gyroposition = 0;
    ModernRoboticsI2cGyro gyro;
    TouchSensor touchSensor;
    OpticalDistanceSensor opticRight = null;
    OpticalDistanceSensor opticLeft = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftMotor  = hardwareMap.dcMotor.get("left motor");
        rightMotor = hardwareMap.dcMotor.get("right motor");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        touchSensor = hardwareMap.touchSensor.get("touch sensor");
        opticLeft = hardwareMap.opticalDistanceSensor.get("optic left");
        opticRight = hardwareMap.opticalDistanceSensor.get("optic right");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        gyro.calibrate();
        while(gyro.isCalibrating() && (opModeIsActive()))
        {
            Thread.sleep(100);
            idle();
        }

        Thread.sleep(5000);
        gyro.resetZAxisIntegrator();

        telemetry.addData("Status", "Gyro Ready");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        turnDegrees(0.2, -40);

        moveStraight(0.3, 200, -45);

        leftMotor.setPower(0.2);
        rightMotor.setPower(0.2);
        while (opModeIsActive() && !(touchSensor.isPressed())) {
            idle();
        }
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

        moveDistance(-0.3, 10);

        turnDegrees(0.2, 20);

        stopAtLine(0.3, 0);


    }

    private void moveStraight(double motorPower, int encoderTicks, int headingFromStart)throws InterruptedException {

        int leftPosition = 0;
        int rightPosition = 0;
        int gyroStart = headingFromStart;//gyro.getIntegratedZValue();
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
                change = (currentGyro - gyroStart)/100.0;
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
                change = (currentGyro - gyroStart)/100.0;
                leftMotor.setPower(Math.min(motorPower + change,0));
                rightMotor.setPower(Math.min(motorPower - change,0));
                idle();
            }
        }


        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

    }
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
    private void turnDegrees (double motorPower, int gyroDegrees)throws InterruptedException {

        gyroposition = gyro.getIntegratedZValue();
        if (gyroDegrees > 0) {
            leftMotor.setPower(motorPower);
            rightMotor.setPower(-motorPower);
            while (opModeIsActive() && (gyro.getIntegratedZValue()<(gyroposition+gyroDegrees))) {
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("degrees", gyro.getIntegratedZValue());
                telemetry.update();

                idle();
            }
        }
        else {
            leftMotor.setPower(-motorPower);
            rightMotor.setPower(motorPower);
            while (opModeIsActive() && (gyro.getIntegratedZValue()>(gyroposition+gyroDegrees))) {
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("degrees", gyro.getIntegratedZValue());
                telemetry.update();

                idle();
            }
        }





        // run until the end of the match (driver presses STOP)

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

    }
    private void stopAtLine (double motorPower, int headingFromStart) throws InterruptedException {

        int gyroStart = headingFromStart;
        int currentGyro;
        double change;

        leftMotor.setPower(motorPower);
        rightMotor.setPower(motorPower);

        while (opModeIsActive() && (opticLeft.getLightDetected() < 0.5 || opticRight.getLightDetected() < 0.5)) {
            currentGyro = gyro.getIntegratedZValue();
            change = (currentGyro - gyroStart)/100.0;
            leftMotor.setPower(Math.max(motorPower - change,0));
            rightMotor.setPower(Math.max(motorPower + change,0));
            idle();
        }

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }
}
