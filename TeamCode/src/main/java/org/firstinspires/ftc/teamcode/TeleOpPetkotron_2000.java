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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;
import static java.lang.Math.sin;

@TeleOp(name="PetkoTron_2000: TeleOp", group="PetkoTron_2000")
//@Disabled
public class TeleOpPetkotron_2000 extends OpMode {
    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HardwarePetkoTron_2000 robot = new HardwarePetkoTron_2000();

    int initPosition = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
        initPosition = robot.arm.getCurrentPosition();
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        // Retract the arm
        /*if (!robot.armZeroLimit.isPressed()) {
            robot.arm.setPower(robot.ARM_DOWN_POWER);
        } else {
            // Stop retracting the arm
            robot.arm.setPower(0);
            //robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }*/
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //double drive = -gamepad1.left_stick_y;
        //double turn  =  gamepad1.right_stick_x;
        //leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        double xInput = gamepad1.left_stick_x;
        double yInput = gamepad1.left_stick_y;
        double zInput = -gamepad1.right_stick_x;

        if (abs(xInput) < 0.1) {
            xInput = 0;
        }
        if (abs(yInput) < 0.1) {
            yInput = 0;
        }
        if (abs(zInput) < 0.1) {
            zInput = 0;
        }

        //Setting the robot to use field-oriented drive
        robot.PetkoTronDrive(xInput, yInput, zInput, false);
        //Controlling the arm (up and down)
        if(gamepad1.y) {
            //if (robot.arm.getCurrentPosition() < robot.armExtendedPosition) {
                robot.arm.setPower(robot.ARM_UP_POWER);
            //}
        } else if(gamepad1.x) {
            // Don't move arm more if the limit switch is pressed
            if (!robot.armZeroLimit.isPressed()) {
                robot.arm.setPower(robot.ARM_DOWN_POWER);
            } else {
                robot.arm.setPower(0);
            }
        } else {
            robot.arm.setPower(0);
        }


        double armPivotPower = (gamepad1.right_trigger - gamepad1.left_trigger)/2;
        if (abs(armPivotPower) < 0.01) {
            robot.armPivot.setPower(0);
        }
        else {
            robot.armPivot.setPower(armPivotPower);
        }

        //Controlling the claw (open and close)
        if(gamepad1.dpad_up) {
            robot.rightClaw.setPosition(robot.rightClaw.getPosition()+0.011);
        }

        if(gamepad1.dpad_down) {
            robot.rightClaw.setPosition(robot.rightClaw.getPosition()-0.01);
        }
        if(gamepad1.dpad_right) {
            robot.leftClaw.setPosition(robot.leftClaw.getPosition()-0.01);
        }

        if(gamepad1.dpad_left) {
            robot.leftClaw.setPosition(robot.leftClaw.getPosition()+0.01);
        }

        if(gamepad1.a) {
            robot.leftIntake.setPower(1);
            robot.rightIntake.setPower(1);
        } else if(gamepad1.b) {
            robot.leftIntake.setPower(-1);
            robot.rightIntake.setPower(-1);
        } else {
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
        }
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        /*telemetry.addData("Current Heading: ", robot.getHeading());
        telemetry.addData("Desired Heading: ", robot.desiredHeading);
        telemetry.addData("Joystick x value:", xInput);
        telemetry.addData("Joystick y value:", yInput);
        telemetry.addData("Joystick z value:", zInput);*/
        telemetry.addData("Arm: ", robot.arm.getCurrentPosition());
        telemetry.addData("Motors", "front left (%.2f), front right (%.2f), rear left (%.2f), rear right (%.2f)", robot.frontLeftPower, robot.frontRightPower, robot.rearLeftPower, robot.rearRightPower);
        telemetry.addData("Right Servo Position",robot.rightClaw.getPosition());
        telemetry.addData("Left Servo Position", robot.leftClaw.getPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        //robot.arm.setTargetPosition(initPosition);
        robot.arm.setPower(0);
        telemetry.addData("Status", "Stopped");
    }

}
