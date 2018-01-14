/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.season2017_2018.EndSeasonCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.season2017_2018.FirstCompCode.MechanumTestBot;


@TeleOp(name="TeleOp End Comp", group="Linear Opmode")
@Disabled
public class TeleOpEndComp extends MechanumTestBotEnd{

    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {

        initializeRobot();
        telemetry.addData("robot init","");
        telemetry.update();
        waitForStart();
        runtime.reset();
        double resetPosTimer = 0;
        while(opModeIsActive())
        {
            moveStraightDrive(gamepad1.right_stick_x, -gamepad1.right_stick_y, gamepad1.left_stick_x);
            telemetry.update();
            if (gamepad1.x)
            {
            }
            else if (gamepad1.a)
            {
                tryToNotFail();
            }
            else if (gamepad1.b)
            {
                setArmUp();
            }

            else if (gamepad1.left_bumper) {
                dumpBlocks(true);
            }
            else if (gamepad1.right_bumper)
            {
                pickUpBlock(true);
            }
            else if (gamepad1.right_trigger > .5)
            {
                setArmDown(true);
            }
            else if (gamepad1.left_trigger > .5)
            {
            }


        }
        stopMotors();

    }
}
