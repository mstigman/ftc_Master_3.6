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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
  This the program for the TeleOp phase. It contains different moves that can be activated via the game
  controllers.
 */

@TeleOp(name="CompTeleOp", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class CompTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor intake;
    DcMotor launcher;
    Servo rightPusher;
    Servo leftPusher;
    Servo ballStopper;
    int endTicks = 0;
    double endTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables.

        leftMotor = hardwareMap.dcMotor.get("left drive");
        rightMotor = hardwareMap.dcMotor.get("right drive");
        intake = hardwareMap.dcMotor.get("intake");
        launcher = hardwareMap.dcMotor.get("launcher");
        rightPusher = hardwareMap.servo.get("right pusher");
        leftPusher = hardwareMap.servo.get("left pusher");
        ballStopper = hardwareMap.servo.get("ball stopper");

        // Set the drive motor directions.
        // "Reverse" the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftPusher.setPosition(.225);
        rightPusher.setPosition(.702);
        ballStopper.setPosition(.45);
        waitForStart();
        runtime.reset();
        double posLeft = 0;
        double posRight = 0;
        double ballLaunched = 0;
        double ballWaiting = 0;
        double holdDelay = 320;
        boolean isOpen = false;
        double launchDelay = 250;
        long counter = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            //Sets the left motor power equal to the left joystick as long as it is higher than 0.08
            //on controller one

            if(Math.abs((gamepad1.left_stick_y)) > .08)
            {
                leftMotor.setPower(gamepad1.left_stick_y);
            }
            else
            {
                leftMotor.setPower(0);
            }

            //Sets the right motor power equal to the left joystick as long as it is higher than 0.08
            //on controlller one
            if(Math.abs((gamepad1.right_stick_y)) > .08) {
                rightMotor.setPower(gamepad1.right_stick_y);
            }
            else
            {
                rightMotor.setPower(0);
            }

            //Runs the intake while the right bumper is pressed on controller two
            if(gamepad1.right_bumper)
            {
                intake.setPower(-1);
            }
            else
            {
                intake.setPower(0);
            }

            //Sets launcher power to high while the left bumper is pressed on controller two
            if(gamepad2.left_bumper) {
                launcher.setPower(-.58);
            }
           // Sets launcher power to medium while the left bumper is pressed on controller two
            else if(gamepad2.right_trigger > .5)
            {
                launcher.setPower(-.53);
            }
            //Sets launcher power to high while the right bumper is pressed
            else if(gamepad2.right_bumper)
            {
                launcher.setPower(-.47);
            }
            else
            {
                launcher.setPower(0);
            }

            //Launches the ball when x is pressed on controller two
            if(gamepad2.x && !isOpen && runtime.milliseconds() > ballWaiting+holdDelay)
            {
                ballLaunched = runtime.milliseconds();
                ballStopper.setPosition(.2);
                isOpen = true;
            }
            if(isOpen && runtime.milliseconds() > ballLaunched+launchDelay)
            {
                ballStopper.setPosition(.45);
                isOpen = false;
                ballWaiting = runtime.milliseconds();
            }

            //Activates the right pusher when a is pressed on controller one
            if(gamepad1.a)
            {
                rightPusher.setPosition(.384);
            }
            else
            {
                rightPusher.setPosition(.702);
            }

            //Activates the left pusher when b is pressed on controller one
            if(gamepad1.b)
            {
                leftPusher.setPosition(.548);
            }
            else
            {
                leftPusher.setPosition(.225);
            }

            idle();

        }
    }
    public void launchControl(double ticksPerSecond)
    {
        double startTicks = launcher.getCurrentPosition();
        double startTime = runtime.seconds();
        double rate;
        double change;
        double power = launcher.getPower();

        if(Math.abs(startTicks - endTicks) > 1)
        {

            rate = (startTicks - endTicks)/(startTime - endTime);
            change = -(rate + ticksPerSecond)/10000;
            if(change > 0)
            {
                change = Math.min(.1,change);
            }
            else
            {
                change = Math.max(-.1,change);
            }
            power += change;
            telemetry.addData("rate",rate);
            telemetry.addData("power",power);
            telemetry.addData("adjust",change);
            telemetry.update();

        }
        else
        {
            if(power == 0)
            {
                power = -.5;
            }
        }
        if(runtime.seconds() - endTime > .1)
        {
            endTime = runtime.seconds();
            endTicks = launcher.getCurrentPosition();
        }
        power = Math.min(power,-.1);
        launcher.setPower(power);

    }
}
