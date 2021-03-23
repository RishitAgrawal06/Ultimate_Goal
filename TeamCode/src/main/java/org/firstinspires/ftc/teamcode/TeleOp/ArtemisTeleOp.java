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

/**
 * This is the TeleOp package
 * **/
package org.firstinspires.ftc.teamcode.TeleOp;

/**
 * Imports OpMode class and the TeleOp declaration
 * **/
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Imports the Artemis Hardware Map Class which initializes all the hardware once the user presses the INIT button
 * **/
import org.firstinspires.ftc.teamcode.HardwareMap.ArtemisHardwareMap;

/**
 * The name attribute specifies the OpMode name on the robot controller and group attribute specifies the group
 * in which that OpMode is located
 * **/
/**
 * This class uses the controller inputs such as joysticks and triggers and dynamically calls various methods in the
 * (imported) Hardware map class once these inputs are called
 * **/
@TeleOp(name = "Artemis TeleOp")
public class ArtemisTeleOp extends OpMode {

    /**
     * The hardware map initialization object which initializes all our motors and servos
     * **/
    ArtemisHardwareMap hardwareMapInitialize = new ArtemisHardwareMap();

    /**
     * This is called ONCE when the driver presses the init button
     * **/
    @Override
    public void init(){
        telemetry.addData("Robot Initialized Successfully in TeleOp"," Wait for hardware to initialize");
        hardwareMapInitialize.init(hardwareMap);
        telemetry.addData("Robot Hardware Initialized Successfully in TeleOp", "Press Play to Start");
    }

    /***
     * This is called ONCE when the driver presses the play button
     * */
    @Override
    public void start(){
        telemetry.addData("Robot in Play Mode in TeleOp","Control the Robot Now");
    }

    /**
     * This is called MULTIPLE TIMES when the driver presses the play button
     * **/
    @Override
    public void loop(){
        /**
         * Gamepad inputs for moving and turning the robot
         * **/
        double leftStickY = -gamepad1.left_stick_y;
        double leftStickX =  gamepad1.left_stick_x * 1.5;
        double rightStickX = gamepad1.right_stick_x;
        if(leftStickY > 0){
            telemetry.addData("Moving","Forwards");
        }else if(leftStickY < 0){
            telemetry.addData("Moving","Backwards");
        }
        else{
            telemetry.addData("Moving", "Not Moving");
        }
        hardwareMapInitialize.moveRobot(leftStickY,leftStickX,rightStickX);

        /**
         * Gamepad inputs for intaking, conveying, and shooting a ring
         * **/
        boolean aFlatButton = gamepad1.a;
        telemetry.addData(aFlatButton ? "Transporting " : "Not Transporting ","Rings");
        hardwareMapInitialize.transportRings(aFlatButton ? 1 : 0);

        boolean xFlatButton = gamepad1.x;
        telemetry.addData(xFlatButton ? "Shooting " :"Not Shooting ", "Rings");
        hardwareMapInitialize.shootRings(xFlatButton ? 1 : 0);

        /**
         * Gamepad inputs for manipulating the hand servo and the arm motor
         * **/
        double positiveArmPower = gamepad1.right_trigger;
        double negativeArmPower = gamepad1.left_trigger;
        boolean loose = gamepad1.right_bumper;

        if(positiveArmPower!=0 || negativeArmPower !=0){
            hardwareMapInitialize.moveArm(positiveArmPower>negativeArmPower ? positiveArmPower : negativeArmPower);
        }else{
            hardwareMapInitialize.moveArm(0);
        }
        hardwareMapInitialize.changeHandPosition(loose ? 0 : 1);
    }

    /**
     * This is called ONCE when the driver presses the stop button and logs that the robot has stopped
     * **/
    @Override
    public void stop(){
        telemetry.addData("Robot has Stopped and Wont Move","Controller Inputs Now Disabled");
    }
}
