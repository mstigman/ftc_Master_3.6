package org.firstinspires.ftc.teamcode.Summer2017;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/*
 Author Salman Mitha
 Two motor robot drive train with 4 sensors.
 Simple joystick based tank drive.
 2 Optical Distance Sensors
 1 color sensor.
 1 gyro sensor.
 Displays the sensor values on the DS phone.
*/

@TeleOp(name="ChrisSensorTestPleaseDoNotDelete", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class ChrisSensorTestPleaseDoNotDelete extends LinearOpMode {

    /* Declare OpMode members. */
       /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    double timestamp;

    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor launcher;

    int gyroposition = 0;
    ModernRoboticsI2cGyro gyro;

    TouchSensor touchSensorRight;
    TouchSensor touchSensorLeft;

    OpticalDistanceSensor opticRight = null;
    OpticalDistanceSensor opticLeft = null;

    ColorSensor colorRight;
    ColorSensor colorLeft;

    Servo rightPusher;
    Servo leftPusher;
    Servo ballStopper;

    ModernRoboticsI2cRangeSensor ultra;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Starting Initializing");
        telemetry.update();

        leftMotor  = hardwareMap.dcMotor.get("left drive");
        rightMotor = hardwareMap.dcMotor.get("right drive");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        touchSensorLeft = hardwareMap.touchSensor.get("touch sensor right");
        touchSensorRight = hardwareMap.touchSensor.get("touch sensor left");
        opticRight = hardwareMap.opticalDistanceSensor.get("optic left");
        opticLeft = hardwareMap.opticalDistanceSensor.get("optic right");
        colorRight = hardwareMap.colorSensor.get("color right");
        colorLeft = hardwareMap.colorSensor.get("color left");
        rightPusher = hardwareMap.servo.get("right pusher");
        leftPusher = hardwareMap.servo.get("left pusher");
        ballStopper = hardwareMap.servo.get("ball stopper");
        launcher = hardwareMap.dcMotor.get("launcher");
        colorLeft.setI2cAddress(I2cAddr.create7bit(0x1e));
        colorRight.setI2cAddress(I2cAddr.create7bit(0x26));
        ultra = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultrasonic");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
     //   rightPusher.setPosition(.702);
       // leftPusher.setPosition(.225);
        //ballStopper.setPosition(.50);
        double segment = 0.25;


        //while (opModeIsActive() && )
        //leftMotor.setTargetPosition((int) segment);
        //rightMotor.setTargetPosition();

        Thread.sleep(3000);

        telemetry.addData("Status", "Gyro Allegedly Calibrated & Initialized");
        telemetry.addData("Gyro"+gyro.getDeviceName(), gyro.getConnectionInfo());
        telemetry.addData("L Col"+colorLeft.getDeviceName(), colorLeft.getConnectionInfo());
        telemetry.addData("R Col"+colorRight.getDeviceName(), colorRight.getConnectionInfo());
        telemetry.addData("Press", "START to begin");
        telemetry.update();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            leftMotor.setPower(-gamepad1.left_stick_y/3.0);
            rightMotor.setPower(-gamepad1.right_stick_y/3.0);

            telemetry.addData("ODS Both", "L %3.2f, R %3.2f", opticLeft.getLightDetected(), opticRight.getLightDetected());
            telemetry.addData("Color L", "R %3d, B %3d, G %3d", colorLeft.red(), colorLeft.blue(), colorLeft.green());
            telemetry.addData("Color R", "R %3d, B %3d, G %3d", colorRight.red(), colorRight.blue(), colorRight.green());
            telemetry.addData("Gyro", "Heading %4d, IntZVal %4d", gyro.getHeading(), gyro.getIntegratedZValue());
            telemetry.addData("left", leftMotor.getCurrentPosition());
            telemetry.addData("right", rightMotor.getCurrentPosition());
            telemetry.addData("raw ultrasonic", ultra.rawUltrasonic());
            telemetry.addData("raw optical", ultra.rawOptical());
            telemetry.addData("cm optical", "%.2f cm", ultra.cmOptical());
            telemetry.addData("cm", "%.2f cm", ultra.getDistance(DistanceUnit.CM));
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
