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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;

@TeleOp(name="PetkoTron_2000: TeleOp", group="PetkoTron_2000")
//@Disabled
public class TeleOpPetkotron_2000 extends OpMode {
    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HardwarePetkoTron_2000 robot = new HardwarePetkoTron_2000();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
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
        // Setup a variable for each drive wheel to save power level for telemetry
        double frontLeftPower = 0;
        double frontRightPower = 0;
        double rearLeftPower = 0;
        double rearRightPower = 0;

        double fb_movement = -gamepad1.left_stick_y;
        double strafe_movement = -gamepad1.left_stick_x;
        double rotation_movement = -gamepad1.right_stick_x;

        // If the y of the left stick is greater (absolute) than the x of the left stick,
        // set power of each motor to the value of the y left stick.
        if(abs(fb_movement) > abs(strafe_movement)) {
            if(abs(fb_movement) <= 0.75) {
                fb_movement = fb_movement*0.2;
            }
            frontLeftPower = fb_movement;
            frontRightPower = fb_movement;
            rearLeftPower = fb_movement;
            rearRightPower = fb_movement;
        }

        if(abs(strafe_movement) > abs(fb_movement)) {
            frontLeftPower = strafe_movement;
            frontRightPower = strafe_movement;
            rearLeftPower = -strafe_movement;
            rearRightPower = -strafe_movement;
        }

        frontLeftPower=Range.clip(frontLeftPower-rotation_movement,-1.0,1.0);
        rearLeftPower=Range.clip(rearLeftPower-rotation_movement,-1.0,1.0);
        frontRightPower=Range.clip(frontRightPower+rotation_movement,-1.0,1.0);
        rearRightPower=Range.clip(rearRightPower+rotation_movement,-1.0,1.0);

        robot.leftFrontDrive.setPower(frontLeftPower);
        robot.rightFrontDrive.setPower(frontRightPower);
        robot.leftBackDrive.setPower(rearLeftPower);
        robot.rightBackDrive.setPower(rearRightPower);

        //double drive = -gamepad1.left_stick_y;
        //double turn  =  gamepad1.right_stick_x;
        //leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        if(gamepad1.dpad_up) {
            robot.arm.setPower(robot.ARM_UP_POWER);
        } else {
            robot.arm.setPower(0);
        }

        if(gamepad1.dpad_down) {
            robot.arm.setPower(robot.ARM_DOWN_POWER);
        } else {
            robot.arm.setPower(0);
        }

        if(gamepad1.left_stick_button) {
            robot.rightClaw.setDirection(Servo.Direction.REVERSE);
            robot.leftClaw.setDirection(Servo.Direction.REVERSE);
        }

        if(gamepad1.right_stick_button) {
            robot.rightClaw.setDirection(Servo.Direction.FORWARD);
            robot.leftClaw.setDirection(Servo.Direction.FORWARD);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "front left (%.2f), front right (%.2f), rear left (%.2f), rear right (%.2f)", frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.arm.setPower(0);
    }

}
