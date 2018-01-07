
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/*
Salman's Notes for this program. ****************
Derived from the Linear Template.
Uses two encoder enabled motors to drive. Objective is to test ecoder based running.

Outline program

Telemetry the start of program.
Initialize Motors, sensors.
Zero encoders at start of program (and start of program only)
Telemetry finishing of initializing and then wait for user to press software START button

Run motors for short duration at constant speed and stop.
Report encoder values during run.

Next block is motors running to a desired position.
Ask user to press a button Y to go to next step, a run to position for one motor turn. (1120 clicks)
Run to position block has following key steps
1. Get current position of encoders to properly set final target position
2. Setup Run to position mode
3. Give desired motor power
3. Give give desired target position (and motors will start)
4. setup a isBusy loop to ensure that motors have enough time to run.

Next block is motors running to a desired position by programmatically counting the clicks
1. Running in using Encoders mode. Setup motor power immediately starts the motors.
2. Program sets a target and then runs a loop while encoder counts don't reach target.
3. Stop motors after encoder target is reached.

The final block of the program just a tank drive with a telemetry output for motor power and encoders.

*/

@TeleOp(name="Sal Test Encoder", group="Linear Opmode")
//@Autonomous(name="Sal Test Encoder", group="Autonomous Opmode")
@Disabled

//Keeping the trial a teleOp mode since it gives us indefinite time to run it. It appears that Autonomous modes
//start with the timer by default.

public class salTestEncoder extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime(); //Is a timer object. Part of FTC SDK
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    OpticalDistanceSensor Light1;

    double leftMotorPower = 0.0;
    double rightMotorPower = 0.0;

    int leftMotorPosition = 0;
    int rightMotorPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        //Sanity Check, letting us know that the program is running
        telemetry.addData("Status:", "Program Initialized"); //NOTE: need TWO strings in telemetry.dddData method)
        telemetry.update(); //This statement clears the telemetry display area and adds the new data.

        //Initializing the objects in the program, i.e. telling the DcMotor object which physical motor it is, which
        // physical sensor it is etc.
        leftMotor  = hardwareMap.dcMotor.get("left motor");
        rightMotor = hardwareMap.dcMotor.get("right motor");
        Light1 = hardwareMap.opticalDistanceSensor.get("Light1");

        // Setting the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // Use FORWARD and reverse in a way that is consistent with hardware team's definition of
        // of the front of the robot.
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        //Resetting the encoders to zero. Do it once over the course of the program. It allegedly is
        // NOT needed all the time.
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle(); //Giving enough time to sent the information to the controller.
        idle(); //Mote time to ensure that all the data is sent to the controller.

        // The following setting will ensure that the motors will run at CONSTANT SPEED as much as possible.
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle(); //Giving the phone some time to ensure that the data is sent to the controller

        //Inform the user about initial values of the motors
        //NOTE: %7d gives seven spaces in the string to output an integer variable.
        // %d will give an output without preset output character width
        //%f will out a float or a double. %7.2f will output a double with
        // 7 width and 2 digits after the decimal.
        telemetry.addData("Status:", "Motors Initialized setup for encoder use");
        telemetry.addData("Initial Encoder Values:",  "Starting at %7d :%7d",
                leftMotor.getCurrentPosition(),
                rightMotor.getCurrentPosition());
        telemetry.addData("Waiting ", "for software PLAY to be pressed");
        telemetry.addData("Next:", "a 2 second run at constant speed, 15 percent power");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY). Start timer
        waitForStart();

        //Start the motors with 15 percent power. Making go back first.
        leftMotor.setPower(-.15);
        rightMotor.setPower(-.15);

        //Want motors to run for a couple of seconds. Safe loop waits for a couple
        // of seconds and informs operator of time elapsed.
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            telemetry.addData("Running for 2 secs: ", "Elapsed time %4.2f s", runtime.seconds());
            telemetry.update();
            idle();
        }

        //Stop the motors
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

        telemetry.addData("Status:", "Stopped motors that ran for 2 seconds");
        telemetry.addData("Final Encoder Values:",  "Ending at %7d :%7d",
                leftMotor.getCurrentPosition(),
                rightMotor.getCurrentPosition());
        telemetry.addData("PRESS", "Y to go to the next step, 1 motor rotation");
        telemetry.addData("Travel 1 motor rotation", "at 0.12 power");
        telemetry.update();

        //wait for Y to be pressed from gamepad 1
        while (opModeIsActive() && (!gamepad1.y)) {
            idle();
        }

        //  CODE BLOCK TO TO RUN TO A FIXED POSITION USING ENCODERS
        //Identify the current position of teh encoders so we can setup the final target.
        leftMotorPosition = leftMotor.getCurrentPosition();
        rightMotorPosition = rightMotor.getCurrentPosition();

        //Set the mode to run to a desired position.
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        idle();

        //Set the desired power during most of the drive. End will be PID
        leftMotor.setPower(.12);
        rightMotor.setPower(.12);

        // Start the motors giving them a target.
        // IMPORTANT NOTE: in the run-to-position mode, motors are started when a target is given.
        // Power is assigned before that. (not sure if the order matters, haven't tested)
        leftMotor.setTargetPosition(leftMotorPosition+1120);
        rightMotor.setTargetPosition(rightMotorPosition+1120);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();
        leftMotor.setPower(0.0); //Stop motors.
        rightMotor.setPower(0.0);

        //Four conditions to break the loop
        //opModeIsActive() is the main safely check. Allow robot to shut down from phone easily
        //runtime.seconds() is a fail safe that that will stop the loop if it goes too long accidentally.
        // probably not needed.
        //leftMotor.isBusy && rightMotor.isBusy is the logical and loop break. When the motor stops running
        // the loop will end.
        //!gamepad1.a si the manual loop break.

        runtime.reset();
        while (opModeIsActive() &&
                (runtime.seconds() < 6.0) &&
                leftMotor.isBusy() && rightMotor.isBusy() &&
                !gamepad1.a) {

            // Display it for the driver.
            telemetry.addData("Target:",  "%7d, %7d, Running: %7d ,%7d",
                    leftMotorPosition+1120,
                    rightMotorPosition+1120,
                    leftMotor.getCurrentPosition(),
                    rightMotor.getCurrentPosition());
            telemetry.addData("Run to Position", "Elapsed time %2.2f s", runtime.seconds());
            telemetry.update();

            idle();  // Allow time for other processes to run.
        }

        // Turn off RUN_TO_POSITION mode. This is VERY IMPORTANT if program has any other
        // and not ending at this point.


        telemetry.addData("DONE", "Press Y to go to Programmer controlled run to position");
        telemetry.addData("Target:",  "%7d, %7d, Running: %7d ,%7d",
                leftMotorPosition+1120,
                rightMotorPosition+1120,
                leftMotor.getCurrentPosition(),
                rightMotor.getCurrentPosition());
        telemetry.addData("Run to Position", "Elapsed time %2.2f s", runtime.seconds());
        telemetry.update();

        //wait for Y to be pressed from gamepad 1
        while (opModeIsActive() && (!gamepad1.y)) {
            idle();
        }

        //Identify the current position of teh encoders so we can setup the final target.
        leftMotorPosition = leftMotor.getCurrentPosition();
        rightMotorPosition = rightMotor.getCurrentPosition();

        //Start motors. Note that motors are in Run-Using-Encoders mode. That means that applying
        // power to the motors will start them.
        leftMotor.setPower(-0.15);
        rightMotor.setPower(-0.15);

        //Several conditions to stop the motors. See above for opModeActive, runtime.seconds, and !gamepad1.a
        //The leftMotor.getCurrentPosition()>(leftMotorPosition-1120) conditions are programmatically checking
        // to see if the encoders have gone beyond the target. If gone beyond, then stop.

        runtime.reset();
        while (opModeIsActive() &&
                (runtime.seconds() < 6.0) &&
                (leftMotor.getCurrentPosition()>(leftMotorPosition-1120)) &&
                (rightMotor.getCurrentPosition()>(rightMotorPosition-1120)) &&
                !gamepad1.a) {

            // Display it for the driver.
            telemetry.addData("Target:",  "%7d, %7d, Running: %7d ,%7d",
                    leftMotorPosition-1120,
                    rightMotorPosition-1120,
                    leftMotor.getCurrentPosition(),
                    rightMotor.getCurrentPosition());
            telemetry.addData("Encoder Running", "Elapsed time %2.2f s", runtime.seconds());
            telemetry.update();

            idle();  // Allow time for other processes to run.
        }

        leftMotor.setPower(0.0); // Stop the motors.
        rightMotor.setPower(0.0);

        telemetry.addData("DONE", "Press Y to go to Programmer controlled run to position");
        telemetry.addData("Target:",  "%7d, %7d, Running: %7d ,%7d",
                leftMotorPosition-1120,
                rightMotorPosition-1120,
                leftMotor.getCurrentPosition(),
                rightMotor.getCurrentPosition());
        telemetry.addData("Encoder Running", "Elapsed time %2.2f s", runtime.seconds());
        telemetry.update();

        //wait for Y to be pressed from gamepad 1
        while (opModeIsActive() && (!gamepad1.y)) {
            idle();
        }

        // Loop to run till drive presses stop. Basic tank drive joystick with loop that mimics a
        //typical TeleOp mode. Reports time and encoder values while running.
        runtime.reset();
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
}
