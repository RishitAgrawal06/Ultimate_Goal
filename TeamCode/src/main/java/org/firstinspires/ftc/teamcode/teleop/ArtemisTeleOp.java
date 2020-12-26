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

package org.firstinspires.ftc.teamcode.teleop;

//imports OpMode class and the TeleOp declaration
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//imports physical hardware to manipulate
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Artemis TeleOp")
public class ArtemisTeleOp extends OpMode {
    /**
     * OpMode members declared
     * **/
    public DcMotor topLeftDriveMotor;
    public DcMotor bottomLeftDriveMotor;
    public DcMotor topRightDriveMotor;
    public DcMotor bottomRightDriveMotor;
    /**
     * This is called ONCE when the driver presses the init button
     * **/
    @Override
    public void init(){
        telemetry.addData("Robot Initialized Successfully in TeleOp"," Wait for hardware to initialize");
        /**
         * Hardware initialized and String Names are in the Configuration File
         * **/
        topLeftDriveMotor = hardwareMap.get(DcMotor.class,"Top-Left-Motor");
        bottomLeftDriveMotor = hardwareMap.get(DcMotor.class, "Bottom-Left-Motor");
        topRightDriveMotor = hardwareMap.get(DcMotor.class, "Top-Right-Motor");
        bottomRightDriveMotor = hardwareMap.get(DcMotor.class, "Bottom-Right-Motor");

        topLeftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /**
         *Since we are putting the motors on different sides we need to reverse direction
         * so that one wheel doesn't pull us backwards
         * **/
        topLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        bottomLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        topRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        bottomRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Robot Hardware Initialized Successfully in TeleOp", "Press Play to Start");
    }
    /***
     * This is called ONCE when the driver presses the play button
     * */
    @Override
    public void start(){
        telemetry.addData("Robot in Play Mode in TeleOp","Get Ready to control");
    }
    /**
     * This is called MULTIPLE TIMES when the driver presses the play button
     * **/
    @Override
    public void loop(){
        topLeftDriveMotor.setPower(0.5);
        bottomLeftDriveMotor.setPower(0.5);
        topRightDriveMotor.setPower(0.5);
        bottomRightDriveMotor.setPower(0.5);
    }
    /**
     * This is called ONCE when the driver presses the stop button
     * **/
    @Override
    public void stop(){
        telemetry.addData("Robot has Stopped and Wont Move","Controller Inputs Now Disabled");
    }
}
