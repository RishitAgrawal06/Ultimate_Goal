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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
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
     * These motors are the intake, conveyor, and shooter motor/servo of the robot
     * **/
    public DcMotor intakeMotor;
    public DcMotor conveyorMotor;
    public DcMotor shooterMotor;
    public CRServo shooterServo;

    /**
     * This is the 1 hand servo and 1 arm motor of the robot
     * **/
    public CRServo handServo;
    public DcMotor armMotor;

    /**
     * DO NOT REMOVE. This hardware map will use the parent hardware map which contains all the names of the parts
     * in which we will use in this class to map and set methods for
     * **/
    HardwareMap hwMap;

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
        shooterServo = hwMap.get(CRServo.class,"Shooter-Servo");

        handServo = hwMap.get(CRServo.class, "Hand-Servo");
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
         * Reverses shooter motor to shoot the correct way and same with the conveyor motor
         * **/
        conveyorMotor.setDirection(DcMotor.Direction.REVERSE);
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
         *The 4 mecanum wheel motors, intake, conveyor, and shooter motor/servo are set to 0 power to keep it from moving when the user presses the INIT button
         * **/
        topLeftDriveMotor.setPower(0);
        bottomLeftDriveMotor.setPower(0);
        topRightDriveMotor.setPower(0);
        bottomRightDriveMotor.setPower(0);

        intakeMotor.setPower(0);
        conveyorMotor.setPower(0);
        shooterMotor.setPower(0);
        shooterServo.setPower(0);

        armMotor.setPower(0);

        /**
         * The 1 servo need to be initialized at the midpoint(0.5) using servo_name.setPosition()
         * **/
        handServo.setPower(1);
    }

    /**
     * This method takes in 3 inputs : Left Stick X/Y and Right Stick X
     * - Left Stick Y moves the robot forwards and backwards(Positive value forwards and Negative value backwards)
     * - Left Stick X introduces strafing to the robot left and right( Positive value makes topLeft and bottomRight motors run which goes right and negative makes bottomLeft and topRight motors move which goes left)
     * -Right Stick X allows the robot to turn left or right(Positive value makes left motors turn more hence going right and negative value makes right motors turn more hence going left)
     * */
    public void moveRobot(double leftStickY, double leftStickX, double rightStickX){
        /**
         * Wheel powers calculated using gamepad 1's inputs leftStickY, leftStickX, and rightStickX
         * **/
        double topLeftPower = leftStickY + leftStickX + rightStickX;
        double bottomLeftPower = leftStickY - leftStickX + rightStickX;
        double topRightPower = leftStickY - leftStickX - rightStickX;
        double bottomRightPower = leftStickY + leftStickX - rightStickX;

        /**
         * Sets the wheel's power 
         * **/
        topLeftDriveMotor.setPower(topLeftPower);
        topRightDriveMotor.setPower(topRightPower);
        bottomLeftDriveMotor.setPower(bottomLeftPower);
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
     * Once the X button is pressed, we set the shooter motor to max power and the shooter servo to max power
     * **/
    public void shootRings(int speed){
        shooterMotor.setPower(speed*0.85);
        shooterServo.setPower(-speed);
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
        handServo.setPower(position);
    }

    /**
     * These are the strictly autonomous methods which are controlled by the timer and wheel/motor selection
     * There is no gamepad influence on this
     * **/

    /**
     * This autonomous move method allows the robot to move forwards and backwards like a tank drive and is controlled by a boolean isForwards
     * **/
    public void autonomousMotorMove(double speed){
        topLeftDriveMotor.setPower(speed);
        topRightDriveMotor.setPower(speed);
        bottomLeftDriveMotor.setPower(speed);
        bottomRightDriveMotor.setPower(speed);
    }

    /**
     * This autonomous strafe method allows the robot to strafe using mecanum wheels and the direction
     * and the direction is specified via a boolean variable
     * **/
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

    /**
     * This autonomous turn method allows the robot to turn with power 0.5 and the direction
     * is specified via boolean variables
     * **/
    public void autonomousMotorTurn(double right, double left){
            topLeftDriveMotor.setPower(left);
            bottomLeftDriveMotor.setPower(left);
            topRightDriveMotor.setPower(right);
            bottomRightDriveMotor.setPower(right);
    }

    /**
     * This autonomous shoot method allows the robot to shoot its rings out.
     * It moves the shooter, conveyor, and intake so that all the rings are transported and shot.
     * **/
    public void autonomousMotorShoot(double speed) throws InterruptedException {
        shooterMotor.setPower(speed);
        Thread.sleep(500);
        conveyorMotor.setPower(speed);
        intakeMotor.setPower(speed);
        shooterServo.setPower(-1);
    }

    /**
     * This autonomous servo handle method allows the servo arm to lift or drop a wobble goal depending on a boolean variable
     * **/
    public void autonomousServoHandle(boolean drop){
        ElapsedTime runtime = new ElapsedTime();
        if(drop){
            while(runtime.seconds()<1.5){
                armMotor.setPower(-0.5);
            }
            while(runtime.seconds()<3.0){
                handServo.setPower(0);
            }
        }
        else{
            while(runtime.seconds()<1.5){
                handServo.setPower(0.5);
            }
            while (runtime.seconds()<1.5){
                armMotor.setPower(0.1);
            }
        }
    }
}
