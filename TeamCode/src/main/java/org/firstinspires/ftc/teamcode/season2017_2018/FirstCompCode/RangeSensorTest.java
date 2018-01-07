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
package org.firstinspires.ftc.teamcode.season2017_2018.FirstCompCode;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
//Dont USE!!
/**
 * {@link RangeSensorTest} illustrates how to use the Modern Robotics
 * Range Sensor.
 *
 * The op mode assumes that the range sensor is configured with a name of "sensor_range".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://modernroboticsinc.com/range-sensor">MR Range Sensor</a>
 */
@Autonomous(name = "Sensor: MR range sensor", group = "Sensor")
@Disabled
public class RangeSensorTest extends LinearOpMode {



    @Override public void runOpMode() throws InterruptedException {


        ElapsedTime runtime = new ElapsedTime();
        double distance = 0;
        double time = 0;
        runtime.reset();
        ModernRoboticsI2cRangeSensor range = null;

        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range finder");

        // wait for the start button to be pressedsedrghuiop[
        waitForStart();

        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 3.0 ){

            //you should probably use getDistance for the log

            distance = range.getDistance(DistanceUnit.CM);
            //Log.d("RangeTest", "Time: , " + runtime.milliseconds());
            Log.d("RangeTest", "time: , " + runtime.milliseconds());
            Log.i("FourthDimension", "distance , " + distance);
        }


        while (opModeIsActive()) {
            telemetry.addData("raw ultrasonic", range.rawUltrasonic());
            telemetry.addData("raw optical", range.rawOptical());
            telemetry.addData("cm optical", "%.2f cm", range.cmOptical());
            telemetry.addData("cm", "%.2f cm", range.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}