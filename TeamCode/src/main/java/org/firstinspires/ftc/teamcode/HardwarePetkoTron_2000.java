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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static java.lang.Math.abs;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case the robot is the PetkoTron_2000, Team 15310's robot.
 */
public class HardwarePetkoTron_2000 {
    //Drive Motors
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;

    //Arm Motor and Claw Servos
    public DcMotor arm = null;
    public Servo leftClaw = null;
    public Servo rightClaw = null;

    //Integrated Gyro and angles to be used in getHeading()
    public BNO055IMU imu;
    private double previousHeading = 0;
    private double integratedHeading = 0;

    public PIDController pidDrive;
    public double desiredHeading = 0;

    //Common positions and power levels for servos and motors
    public static final double INITIAL_CLAW = 1.0;
    public static final double ARM_UP_POWER = 1.0;
    public static final double ARM_DOWN_POWER = -0.5;
    public double frontLeftPower = 0;
    public double frontRightPower = 0;
    public double rearLeftPower = 0;
    public double rearRightPower = 0;


    /* local OpMode members. */
    HardwareMap hwMap =  null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public HardwarePetkoTron_2000(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        //Save reference to Hardware map
        hwMap = ahwMap;

        //Initializing the gyro and setting the parameters for recording the angles
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        // Set up parameters for driving in a straight line.
        pidDrive = new PIDController(.05, 0, 0);
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, 1);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        //Define and initialize motors
        leftFrontDrive = hwMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hwMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hwMap.get(DcMotor.class, "right_back_drive");
        arm = hwMap.get(DcMotor.class, "arm");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.REVERSE);

        //Set all motors to zero power
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        arm.setPower(0);

        //Set all motors to run with encoders.
        //May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Define and initialize ALL installed servos.
        leftClaw  = hwMap.get(Servo.class, "left_claw");
        rightClaw = hwMap.get(Servo.class, "right_claw");
        leftClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setDirection(Servo.Direction.REVERSE);
        leftClaw.setPosition(INITIAL_CLAW);
        rightClaw.setPosition(INITIAL_CLAW);
    }

    public void resetEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Getting and returning the current header (goes from (-inf, inf) as opposed to (-180, 180))
    public double getHeading() {
        double currentHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double deltaHeading = currentHeading - previousHeading;

        //Shifting the heading so that it doesn't display as between -180 and 180
        if (deltaHeading < -180) {
            deltaHeading += 360;
        } else if (deltaHeading >= 180) {
            deltaHeading -= 360;
        }
        integratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return integratedHeading;
    }


    //General method for actually driving the robot. Can use field-oriented drive or robot-oriented drive.
    public void PetkoTronDrive(double xInput, double yInput, double zInput, boolean fieldDrive) {


        // Use PID with imu input to drive in a straight line.
        // https://stemrobotics.cs.pdx.edu/node/7268
        double correction = 0;
        if (zInput == 0) {
            // Apply PID correction if we don't want to rotate
            //correction = pidDrive.performPID(getHeading() - desiredHeading);
        } else {
            // Save the current heading as the desired heading if we are rotating
            // Also, don't apply a correction
            desiredHeading = getHeading();
        }

        //Values for moving
        double fb_movement;
        double strafe_movement;
        double rotation_movement = -zInput + correction;

        double heading = getHeading();
        /*if (getHeading() < 20 && getHeading() > -20) {
            heading = 0;
        }*/
        //Setting variables for using field-oriented drive
        if(fieldDrive) {
            double forward = Range.clip(-yInput, -1.0, 1.0);
            double strafe = Range.clip(-xInput, -1.0, 1.0);
            double gyroRadians = heading * (Math.PI/180);

            fb_movement = (forward * Math.cos(gyroRadians)) + (strafe * Math.sin(gyroRadians));
            strafe_movement = (-forward * Math.sin(gyroRadians)) + (strafe * Math.cos(gyroRadians));
        } else { //Using robot-oriented drive
            fb_movement = yInput;
            strafe_movement = xInput;
        }

        //If the y of the left stick is greater (absolute) than the x of the left stick,
        //set power of each motor to the value of the y left stick.
        /*if(abs(fb_movement) > abs(strafe_movement)) {
            /*if(abs(fb_movement) <= 0.75) {
                fb_movement = fb_movement*0.5;
            }
            frontLeftPower = fb_movement;
            frontRightPower = fb_movement;
            rearLeftPower = fb_movement;
            rearRightPower = fb_movement;
        } else {
            frontLeftPower = -strafe_movement;
            frontRightPower = strafe_movement;
            rearLeftPower = strafe_movement;
            rearRightPower = -strafe_movement;
        }*/


        frontLeftPower = fb_movement-strafe_movement;
        frontRightPower = fb_movement+strafe_movement;
        rearLeftPower = fb_movement+strafe_movement;
        rearRightPower = fb_movement-strafe_movement;

        frontLeftPower=Range.clip(frontLeftPower-rotation_movement,-0.5,0.5);
        rearLeftPower=Range.clip(rearLeftPower-rotation_movement,-0.5,0.5);
        frontRightPower=Range.clip(frontRightPower+rotation_movement,-0.5,0.5);
        rearRightPower=Range.clip(rearRightPower+rotation_movement,-0.5,0.5);

        leftFrontDrive.setPower(frontLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        leftBackDrive.setPower(rearLeftPower);
        rightBackDrive.setPower(rearRightPower);
    }
 }

