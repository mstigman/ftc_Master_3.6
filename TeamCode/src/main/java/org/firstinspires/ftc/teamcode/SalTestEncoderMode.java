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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 Testing to see if we can run using encoders in fixed power mode.
 i.e. check the value of the encoders when motors setmode is RUN_WITHOUT_ENCODER
 this program should demonstrate whether or not when can use encoder when
 the motors are running at constant power.
 The program also has a method to run the motor and slow down. Use it for graduated stops.
 Then runs
 */

@TeleOp(name="SalTestEncoderMode", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class SalTestEncoderMode extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;

    private double leftMotorPower;
    private double rightMotorPower;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status: ", "Started");
        telemetry.update();

        //Link the motor objects with the hardware
        leftMotor  = hardwareMap.dcMotor.get("left motor");
        rightMotor = hardwareMap.dcMotor.get("right motor");

        // Set the drive motor directions:
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status: ", "Motors Initialized");
        telemetry.addData("Press:", "Play to start");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        moveToAndSlow(0.17,400,0.0);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        idle();
        idle();

        moveToAndSlow(-0.17,400,0.0);


        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            // Slow it down by factor or 4
            leftMotorPower = -gamepad1.left_stick_y/4.0;
            rightMotorPower = -gamepad1.right_stick_y/4.0;

            leftMotor.setPower(leftMotorPower);
            rightMotor.setPower(rightMotorPower);

            telemetry.addData("Joystick running", "Run Time: %4.2f" + runtime.seconds());
            telemetry.addData("Motor Power:",  "%4.2f   :%4.2f",
                    leftMotorPower,
                    rightMotorPower);
            telemetry.addData("Encoder Val:",  "%7d :%7d",
                    leftMotor.getCurrentPosition(),
                    rightMotor.getCurrentPosition());
            telemetry.update();

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

    }

    // Moves the robot at a certain speed and then slows it down. Use final power 0 to stop. Use it to have gradual stop
    //  set direction of the robot with motor power
    private void moveToAndSlow(double motorPower, int distanceToTravel, double finalmotorPower) throws InterruptedException {

        int leftTarget;
        int rightTarget;

        int motorDirection;

        distanceToTravel = Math.abs(distanceToTravel);
        motorDirection = (int)Math.signum(motorPower);

        leftTarget = leftMotor.getCurrentPosition() + (motorDirection*distanceToTravel);
        rightTarget = rightMotor.getCurrentPosition() + (motorDirection*distanceToTravel);

        leftMotor.setPower(motorPower);
        rightMotor.setPower(motorPower);
        while(opModeIsActive() &&
                (leftMotor.getCurrentPosition() * motorDirection) < (motorDirection * leftTarget) &&
                (rightMotor.getCurrentPosition() * motorDirection) < rightTarget * motorDirection){
            idle();
        }
        leftMotor.setPower(finalmotorPower);
        rightMotor.setPower(finalmotorPower);

    }
}
