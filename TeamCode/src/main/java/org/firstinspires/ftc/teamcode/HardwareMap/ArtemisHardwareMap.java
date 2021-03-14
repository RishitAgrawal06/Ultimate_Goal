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
 * This is the HardwareMap package
 * **/
package org.firstinspires.ftc.teamcode.HardwareMap;

/**
 *Imports physical hardware to manipulate
 * **/
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This program is not an OpMode. Instead it initialize all the hardware including motors and servos and
 * provides various methods on how to get it to move and turn(A helper class)
 * **/
public class ArtemisHardwareMap {

    /**
     * These motors are the 4 mecanum wheels of our robot
     * **/
    public DcMotor topLeftDriveMotor;
    public DcMotor bottomLeftDriveMotor;
    public DcMotor topRightDriveMotor;
    public DcMotor bottomRightDriveMotor;

    /**
     * These motors are the intake, conveyor, and shooter of the robot
     * **/
    public DcMotor intakeMotor;
    public DcMotor conveyorMotor;
    public DcMotor shooterMotor;

    /**
     * These are the 1 hand servo and 1 arm motor of the robot
     * **/
    public Servo handServo;
    public DcMotor armMotor;

    /**
     * DO NOT REMOVE. This hardware map will use the parent hardware map which contains all the names of the parts
     * in which we will use in this class to map and set methods for
     * **/
    HardwareMap hwMap;

    /**
     * Constructor for the ArtemisHardwareMap method in case we need it
     * **/
    public ArtemisHardwareMap(){

    }

    /**
     * This method initializes all the motors and servos using the parent hardware map
     * **/
    public void init(HardwareMap ahwMap) {

        /**
         * Assigns the parent hardware map to local ArtemisHardwareMap class variable
         * **/
        hwMap = ahwMap;

        /**
         * Hardware initialized and String Names are in the Configuration File for Hardware Map
         * **/
        topLeftDriveMotor = hwMap.get(DcMotor.class,"Top-Left-Motor");
        bottomLeftDriveMotor = hwMap.get(DcMotor.class, "Bottom-Left-Motor");
        topRightDriveMotor = hwMap.get(DcMotor.class, "Top-Right-Motor");
        bottomRightDriveMotor = hwMap.get(DcMotor.class, "Bottom-Right-Motor");

        intakeMotor = hwMap.get(DcMotor.class, "Intake-Motor");
        conveyorMotor = hwMap.get(DcMotor.class,"Conveyor-Motor");
        shooterMotor = hwMap.get(DcMotor.class,"Shooter-Motor");

        handServo = hwMap.get(Servo.class, "Hand-Servo");
        armMotor = hwMap.get(DcMotor.class, "Arm-Motor");
        /**
         * Allow the 4 wheel motors to be run without encoders since we are doing a time based autonomous
         * **/
        topLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /**
         * No need for the intake, conveyor and shooter motors to track rotations so we run it without encoders
         * **/
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        conveyorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /**
         * No need for encoders for the arm motor since we just drop and lift with it
         * **/
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /**
         *Since we are putting the motors on different sides we need to reverse direction so that one wheel doesn't pull us backwards
         * **/
        topLeftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        bottomLeftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        topRightDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        bottomRightDriveMotor.setDirection(DcMotor.Direction.FORWARD);

        /**
         * Reverses shooter motor to shoot the correct way
         * **/
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        /**
         * We are setting the motor 0 mode power to be brake as it actively stops the robot and doesn't rely on the surface to slow down once the robot power is set to 0
         * **/
        topLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /**
         * Intake, conveyor, and shooter motors set to brake as there is no surface for it to slow down for float
         * **/
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /***
         * Arm motor will set to break as there is no friction in the air
         * */
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /**
         *The 4 mecanum wheel motors, intake, conveyor, and shooter are set to 0 power to keep it from moving when the user presses the INIT button
         * **/
        topLeftDriveMotor.setPower(0);
        bottomLeftDriveMotor.setPower(0);
        topRightDriveMotor.setPower(0);
        bottomRightDriveMotor.setPower(0);

        intakeMotor.setPower(0);
        conveyorMotor.setPower(0);
        shooterMotor.setPower(0);

        armMotor.setPower(0);

        /**
         * The 1 servo need to be initialized at the midpoint(0.5) using servo_name.setPosition()
         * **/
        handServo.setPosition(1);
    }

    /**
     * This method takes in 3 inputs : Left Stick X/Y and Right Stick X
     * - Left Stick Y moves the robot forwards and backwards(Positive value forwards and Negative value backwards)
     * - Left Stick X introduces strafing to the robot left and right( Positive value makes topLeft and bottomRight motors run which goes right and negative makes bottomLeft and topRight motors move which goes left)
     * -Right Stick X allows the robot to turn left or right(Positive value makes left motors turn more hence going right and negative value makes right motors turn more hence going left)
     * */
    public void moveRobot(double leftStickY, double leftStickX, double rightStickX){
        /**
         * Wheel powers calculated using gamepad inputs leftStickY, leftStickX, and rightStickX
         * **/
        double topLeftPower = leftStickY + leftStickX + rightStickX;
        double bottomLeftPower = leftStickY - leftStickX + rightStickX;
        double topRightPower = leftStickY - leftStickX - rightStickX;
        double bottomRightPower = leftStickY + leftStickX - rightStickX;

        /**
         * If any value is greater than 1 then lets scale the power all the wheels from -1 to 1
         * **/
        if (Math.abs(topLeftPower) > 1 || Math.abs(bottomLeftPower) > 1 ||
                Math.abs(topRightPower) > 1 || Math.abs(bottomRightPower) > 1 ) {
            // Find the largest power
            double max = 0;
            max = Math.max(Math.abs(topLeftPower), Math.abs(bottomLeftPower));
            max = Math.max(Math.abs(topRightPower), max);
            max = Math.max(Math.abs(bottomRightPower), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            topLeftPower /= max;
            bottomLeftPower /= max;
            topRightPower /= max;
            bottomRightPower /= max;
        }

        /**
         * Sets the wheel's power once scaled
         * **/
        topLeftDriveMotor.setPower(topLeftPower);
        bottomLeftDriveMotor.setPower(bottomLeftPower);
        topRightDriveMotor.setPower(topRightPower);
        bottomRightDriveMotor.setPower(bottomRightPower);
    }

    /**
     * This method takes in 1 input : the A button
     * Once the A button is pressed, we set the intake and conveyor motors to max power
     * **/
    public void transportRings(int speed){
        intakeMotor.setPower(speed);
        conveyorMotor.setPower(speed);
    }

    /**
     * This method takes in 1 input: the X button
     * Once the X button is pressed, we set the shooter motor to max power
     * **/
    public void shootRings(int speed){
        shooterMotor.setPower(speed);
    }

    /**
     * This method takes in 2 inputs : left and right trigger
     * Whichever trigger power is greater is the one that will move forwards/backwards
     * **/
    public void moveArm(double speed){
        armMotor.setPower(speed);
    }

    /**
     * This method takes in 1 input: right bumper
     * If the right bumper is pressed then the servo hand will loosen else it will tighten
     * **/
    public void changeHandPosition(int position){
        handServo.setPosition(position);
    }

    /**
     * These are the strictly autonomous methods which are controlled by the timer and wheel/motor selection
     * There is no gamepad influence on this
     * **/
    public void autonomousMotorMove(boolean isForward){
        double speed = isForward ? 0.5 : -0.5;
        topLeftDriveMotor.setPower(speed);
        bottomLeftDriveMotor.setPower(speed);
        topRightDriveMotor.setPower(speed);
        bottomRightDriveMotor.setPower(speed);
    }

    public void autonomousMotorStrafe(boolean topLeft, boolean bottomLeft,boolean topRight,boolean bottomRight){
        if(topLeft){
            topLeftDriveMotor.setPower( 0);
            bottomLeftDriveMotor.setPower(0.5);
            topRightDriveMotor.setPower(0.5);
            bottomRightDriveMotor.setPower( 0);
        }
        else if(bottomLeft){
            topLeftDriveMotor.setPower(-0.5);
            bottomLeftDriveMotor.setPower(0);
            topRightDriveMotor.setPower(0);
            bottomRightDriveMotor.setPower(-0.5);
        }
        else if(topRight){
            topLeftDriveMotor.setPower(0.5);
            bottomLeftDriveMotor.setPower(0);
            topRightDriveMotor.setPower(0);
            bottomRightDriveMotor.setPower(0.5);
        }
        else if(bottomRight){
            topLeftDriveMotor.setPower( 0);
            bottomLeftDriveMotor.setPower(-0.5);
            topRightDriveMotor.setPower(-0.5);
            bottomRightDriveMotor.setPower( 0);
        }
    }

    public void autonomousMotorTurn(boolean right, boolean left){
        double turnPower = 0.5;
        if(right){
            topLeftDriveMotor.setPower(turnPower);
            bottomLeftDriveMotor.setPower(turnPower);
            topRightDriveMotor.setPower(0);
            bottomRightDriveMotor.setPower(0);
        }else if(left){
            topLeftDriveMotor.setPower(0);
            bottomLeftDriveMotor.setPower(0);
            topRightDriveMotor.setPower(turnPower);
            bottomRightDriveMotor.setPower(turnPower);
        }
    }

    public void autonomousMotorShoot(){
        shooterMotor.setPower(1);
        conveyorMotor.setPower(1);
        intakeMotor.setPower(1);
    }

    public void autonomousServoHandle(boolean drop){
        ElapsedTime runtime = new ElapsedTime();
        if(drop){
            while(runtime.seconds()<1.5){
                armMotor.setPower(-0.1);
            }
            while(runtime.seconds()<3.0){
                handServo.setPosition(0);
            }
        }
        else{
            while(runtime.seconds()<1.5){
                handServo.setPosition(1);
            }
            while (runtime.seconds()<1.5){
                armMotor.setPower(0.1);
            }
        }
    }
}
