/*
Copyright 2022 FIRST Tech Challenge Team 4998

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

@Autonomous
@Disabled
public class ArbenAutoBlueRight extends LinearOpMode {
    
    DcMotorEx m1, m2, m3, m4;
    BNO055IMU imu;
    
    private Servo rotate;
    private Servo grabber;
    
    private DistanceSensor sensorDistance;

    private void drive(double py, double px, double pa) {
                
        //py is the power of the left stick on the y axis (vertical movement)
        // Down to Up, -1.00 to 1.00
        
        //px is the power of the left stick on the x axis (horizontal movement)
        // Left to Right, -1.00 to 1.00
        
        //pa is the power of the body rotation
        
        if (Math.abs(pa) < 0.05) pa = 0;
        double p1 = -px + py + pa;
        double p2 = px + py - pa;
        double p3 = px + py + pa;
        double p4 = -px + py - pa;
        double max = Math.max(1.0, Math.abs(p1));
        max = Math.max(max, Math.abs(p2));
        max = Math.max(max, Math.abs(p3));
        max = Math.max(max, Math.abs(p4));
        p1 /= max;
        p2 /= max;
        p3 /= max;
        p4 /= max;
        m1.setPower(p1);
        m2.setPower(p2);
        m3.setPower(p3);
        m4.setPower(p4);
    }
    

    @Override
    public void runOpMode() {
        
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        
        rotate = hardwareMap.get(Servo.class, "rotate");
        grabber = hardwareMap.get(Servo.class, "grabber");
    
        grabber.setPosition(1);
        rotate.setPosition(0.9);
        
        boolean hasSeen = false;
        
        m1 = hardwareMap.get(DcMotorEx.class, "frontl");
        m2 = hardwareMap.get(DcMotorEx.class, "backl");
        m3 = hardwareMap.get(DcMotorEx.class, "frontr");
        m4 = hardwareMap.get(DcMotorEx.class, "backr");
        
        //m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        //m3.setDirection(DcMotor.Direction.REVERSE);
        m4.setDirection(DcMotor.Direction.REVERSE);
        
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //DRIVING MEASUREMENTS
        /*
        * One square to another is ~700ms
        * Rotating 90 degrees is ~800ms
        * 
        */
        drive(0,0.3,0);
        sleep(600);
        
        drive(0,0,0);
        sleep(500);
        if((sensorDistance.getDistance(DistanceUnit.INCH)/12) <= 3){ //Pos 1 check
            
            telemetry.addData("Position Found", 1);
            telemetry.update();
            
            rotate.setPosition(0.65); //Move rotator to the ground
            sleep(350);
            
            drive(0,-0.5,0); //Align with object (Line)
            sleep(200);
            
            drive(0.5,0,0); //Drive up to the line
            sleep(415);
            
            drive(0,0,-0.5);
            sleep(20);
            
            drive(0,0,0);
            sleep(400);
            grabber.setPosition(0.6); //Release hand
            sleep(1000);
            rotate.setPosition(1);
            sleep(1000);
            
        }
    }
}
