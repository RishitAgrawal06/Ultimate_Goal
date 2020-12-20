package org.firstinspires.ftc.teamcode.autonomous;

//imports OpMode class and the TeleOp symbol
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class ArtemisAutonomous extends OpMode {
    @Override
    public void init(){
        telemetry.addData("Robot initialized in Autonomous", "Press play to start");
    }
    @Override
    public void loop(){
        telemetry.addData("Robot Status Autonomous: ", "Is in Play Mode");
    }
}
