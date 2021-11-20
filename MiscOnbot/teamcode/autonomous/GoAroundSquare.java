package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.*;

@Autonomous(name="Go around a red square", group = "Test")

public class GoAroundSquare extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        // Post update
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        DcMotor left_drive        = hardwareMap.get(DcMotor.class, "Left");
        DcMotor right_drive      = hardwareMap.get(DcMotor.class, "Right");
        
        ColorSensor color_sensor = hardwareMap.get(ColorSensor.class, "Color_Sensor");
        
        // Set up classes
        DriveTrain drive = new DriveTrain(left_drive, right_drive);
        ColorDetection color = new ColorDetection(color_sensor);
        
        // Wait for game to start
        waitForStart();
        runtime.reset();
        
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            if (color.checkForRed()){
                drive.driveJoystick(.1, -.3);
            } else {
                drive.driveJoystick(-.3, 0);
            }
            
            
            // Telemetry
            //color.recordIndividualValues();
            telemetry.addData("Status", "Run Time: " + runtime.toString());
        }
    }
    
}