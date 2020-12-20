package org.firstinspires.ftc.teamcode.teleop;

//imports OpMode class and the TeleOp declaration
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ArtemisTeleOp extends OpMode {
    /**
     * This is called when the driver presses the init button
     * **/
    @Override
    public void init(){
        telemetry.addData("Robot Initialized Successfully in TeleOp","Press Play to Start");
    }
    /**
     * This is called when the driver presses the play button
     * **/
    @Override
    public void loop(){
        telemetry.addData("Robot Status TeleOp:", "Is in Play Mode");
    }
}
