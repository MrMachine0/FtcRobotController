/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Modified by clebron
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 */

@Autonomous(name="WalkerBot: Auto Drive By Distance", group="WalkerBot")
//@Disabled
public class AutoDriveByDistance_LinearTemplate extends LinearOpMode {

    /* Declare OpMode members. */
    WalkerBaseRobot         robot   = new WalkerBaseRobot();   // Use a Pushbot's hardware
    private double          COUNTS_PER_MOTOR_REV = 2150.8;
    private final double    DRIVE_GEAR_REDUCTION = 1.0;
    private double          WHEEL_DIAMETER_INCHES = 3.75;
    private double          COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private double          COUNTS_PER_DEGREE = COUNTS_PER_MOTOR_REV / 360;
    private double          DRIVE_SPEED = 0.6;
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;
    

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way
        
        // Step 1:  Drive forward for 3 seconds
        

        // Step 2:  Spin right for 1.3 seconds


        // Step 3:  Drive Backwards for 1 Second


        // Step 4:  Stop and close the claw.

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    // Functions for Autonomous Based on FTC Labs with Arduino
            
    public void driveStraight(double p, double t) {
        robot.leftMotor.setPower(p);
        robot.rightMotor.setPower(p);
        delay(t);
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    public void driveStraight(double p) {
        robot.leftMotor.setPower(p);
        robot.rightMotor.setPower(p);
    }

    public void pointTurn(double p, double t) {
        robot.leftMotor.setPower(p);
        robot.rightMotor.setPower(-p);
        delay(t);
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    public void pointTurn(double p) {
        robot.leftMotor.setPower(p);
        robot.rightMotor.setPower(-p);
    }

    /*public void openHands() {  // You will need to modify this
        robot.claw.setPosition(1.0);
    }

    public void closeHands() { // You will need to modify this
        robot.claw.setPosition(0.0);
    }*/

    public void delay(double t) { // Imitates the Arduino delay function
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    /*public void moveArm(double p, double t) {
        robot.armMotor.setPower(p);
        delay(t);
        robot.armMotor.setPower(0);
    }*/

    // Add more functions here if needed
    public void driveStraightInches(double speed,
                             double inches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Reverse inches
        //inches = inches * -1;

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set to Limit of DRIVE_SPEED
        if (Math.abs(speed) > DRIVE_SPEED) {
            speed = DRIVE_SPEED; //
        }

        // Ensure that the opmode is still active
        //if (opModeIsActive()) {
        if (true) {       // Swapped out to include in BaseRobot

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(0.9*Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {
                   // Wait for Sequence to complete
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void correctedDrive(double inches) {
        double speed = DRIVE_SPEED;
        double timeoutS = Math.abs(inches) * 0.03;
        driveStraightInches(speed, inches, timeoutS);
    }
    public void pointTurnDegrees(double speed,
                                 double deg,
                                 double timeoutS) {

        int newLeftTarget;
        int newRightTarget;

        // Reverse inches
        deg = deg * -1;

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set to Limit of DRIVE_SPEED
        if (Math.abs(speed) > TURN_SPEED) {
            speed = TURN_SPEED; //
        }

        // Ensure that the opmode is still active
        //if (opModeIsActive()) {
        if (true) {       // Swapped out to include in MaristBaseRobot

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(deg * COUNTS_PER_DEGREE);
            newRightTarget = robot.rightMotor.getCurrentPosition() - (int)(deg * COUNTS_PER_DEGREE);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {
                // Wait for Sequence to complete
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
