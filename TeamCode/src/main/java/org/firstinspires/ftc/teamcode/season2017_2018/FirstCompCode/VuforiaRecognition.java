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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.season2017_2018.EndSeasonCode.MechanumTestBotEnd;

import java.util.ArrayList;
import java.util.List;

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

@Autonomous(name="Concept: Vuforia Navigation", group ="Concept")
@Disabled
public class VuforiaRecognition extends LinearOpMode {

    public static final String TAG = "Vuforia Sample";

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer.Parameters params;
    VuforiaTrackables visionTarget;
    VuforiaTrackable target;
    VuforiaTrackableDefaultListener DefaultListener;
    ElapsedTime runtime = new ElapsedTime();

    OpenGLMatrix getLastLocation;
    OpenGLMatrix phoneLocation;
    int VuMarkCenter = 2;
    int VuMarkRight = 3;
    int VuMarkLeft = 1;


    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AfWRMZn/////AAAAGeL3ADUJQUPGhNAhNQUth5aLwwOlSO08asw1uqJQF1ndzliHZxf3aUyFBY7KSvgH1zxdJlE+WRbhM/HwOddafzk+SEMQjJ+u6udl9ooyIAHjK6Vh3GBu5aYDYUoe2gM0/7mMvEy+4OAGlhvQ6ZYdRlBVc2FeK1jJsW9eeELZC/i5fhoSnQtWKBiD8YDQngbRTz8SHJGfWbJCCKg+c6QRpNRFWmRiwpjh0hIN+GeyZLw6GR3vcRl3b6vPZEBrwK018Tq98jRY/le3z5egiVmP6+VE/Vuw6LN7GekcATsB9n6hM2ukKhWswp6ZbzTVwSW5+0PJrr78t6OCdA4r0JMVyi9OUlbz4SC+LR+4b23BQzz9";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        //VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
       // VuforiaTrackable relicTemplate = relicTrackables.get(0);
        //relicTemplate.setName("relicVuMarkTemplate"); //ThIs MiGhT help in DEBUGging but it's not REALLY necessary

        telemetry.update();
        waitForStart();
        int vuf = VuforiaInt();
        telemetry.addData("vuf saw: ", vuf);
        telemetry.update();
        while (opModeIsActive())
        {
            idle();
        }
       // relicTrackables.activate();

        while (opModeIsActive()) {

           RelicRecoveryVuMark vuMark = null;// RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

    /* Found an instance of the template. In the actual game, you will probably
     * loop until this condition occurs, then move on to act accordingly depending
     * on which VuMark was visible. */
                //               telemetry.addData("VuMark", "%s visible", vuMark);
                //               telemetry.update();

                if (vuMark == RelicRecoveryVuMark.CENTER) {
                    telemetry.addData("VuMark", "%s visible", VuMarkCenter);
                    telemetry.update();
                }

                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    telemetry.addData("VuMark", "%s visible", VuMarkLeft);
                    telemetry.update();
                }

                if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    telemetry.addData("VuMark", "%s visible", VuMarkRight);
                    telemetry.update();
                }
                telemetry.update();
            }
        }
    }

    private int VuforiaInt() {
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTrackables.activate();
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        int VuMarkLeft = 1;
        int VuMarkCenter = 2;
        int VuMarkRight = 3;
        runtime.reset();
        while(opModeIsActive() && runtime.milliseconds() < 3000) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                if (vuMark == RelicRecoveryVuMark.LEFT){
                    return 1;
                }
                if (vuMark == RelicRecoveryVuMark.CENTER){
                    return 2;
                }
                if (vuMark == RelicRecoveryVuMark.RIGHT){
                    return 3;
                }
            }

        }
        return 0;
    }




    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }
}
