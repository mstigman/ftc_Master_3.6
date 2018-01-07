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
import com.qualcomm.robotcore.hardware.Servo;
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

@TeleOp(name="CompTeleOpChrisTest", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class CompTeleOpChrisTest extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor intake;
    DcMotor launcher;
    Servo rightPusher;
    Servo leftPusher;
    Servo ballStopper;


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
        ballStopper.setPosition(.2);
        waitForStart();
        runtime.reset();
        double posLeft = 0;
        double posRight = 0;
        double ballLaunched = 0;
        double ballWaiting = 0;
        double launchDelay = 200;
        boolean isOpen = false;
        double stopDelay = 375;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            if(Math.abs((gamepad1.left_stick_y)) > .08)
            {
                leftMotor.setPower(gamepad1.left_stick_y);
            }
            else
            {
                leftMotor.setPower(0);
            }

            if(Math.abs((gamepad1.right_stick_y)) > .08) {
                rightMotor.setPower(gamepad1.right_stick_y);
            }
            else
            {
                rightMotor.setPower(0);
            }

            if(gamepad1.right_bumper)
            {
                intake.setPower(-1);
            }
            else
            {
                intake.setPower(0);
            }

            if(gamepad2.left_bumper) {
                launcher.setPower(-.8);
            }
            else if(gamepad2.right_trigger > .5)
            {
                launcher.setPower(-.65);
            }
            else if(gamepad2.right_bumper)
            {
                launcher.setPower(-.50);
            }
            else
            {
                launcher.setPower(0);
            }

            if(gamepad2.x && !isOpen && runtime.milliseconds() > ballWaiting+launchDelay)
            {
                ballLaunched = runtime.milliseconds();
                ballStopper.setPosition(.2);
                isOpen = true;
            }

            if(isOpen && runtime.milliseconds() > ballLaunched+stopDelay)
            {
                ballStopper.setPosition(.45);
                isOpen = false;
                ballWaiting = runtime.milliseconds();
            }

            if(gamepad1.a)
            {
                rightPusher.setPosition(.384);
            }
            else
            {
                rightPusher.setPosition(.702);
            }

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
}
