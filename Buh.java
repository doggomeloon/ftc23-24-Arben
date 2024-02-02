package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
//import org.firstinspires.ftc.robotcore.util.ElapsedTime;

@Autonomous

public class Buh extends LinearOpMode {
    
    ColorSensor sensorColor;
    
    DcMotorEx m1, m2, m3, m4;
    
    private Servo rotate;
    private Servo grabberLeft, grabberRight;
    //private elapsedTime runtime = new ElapsedTime();
    
    private DistanceSensor DSFL, DSL, DSR, DSFR;
    
    private boolean hasSeenLine = false;

    private void drive(double py, double px, double pa) {
                
        //py is the power of the left stick on the y axis (vertical movement)
        // Down to Up, -1.00 to 1.00
        
        //px is the power of the left stick on the x axis (horizontal movement)
        // Left to Right, -1.00 to 1.00
        
        //pa is the power of the body rotation
        
        if (Math.abs(pa) < 0.05) pa = 0;
        double p1 = px + py + pa; //fl
        double p2 = -px + py + pa; //bl
        double p3 = -px + py - pa; //fr
        double p4 = px + py - pa; //br
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
    
    private void stopDrive(){
        drive(0,0,0);
        sleep(400);
    }
    

    @Override
    public void runOpMode() {
        
        DSFL = hardwareMap.get(DistanceSensor.class, "distanceSensorFrontLeft");
        DSL = hardwareMap.get(DistanceSensor.class, "distanceSensorLeft");
        DSR = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");
        DSFR = hardwareMap.get(DistanceSensor.class, "distanceSensorFrontRight");
        
        sensorColor = hardwareMap.get(ColorSensor.class, "Color");
        
        //Grabber motors
        rotate = hardwareMap.get(Servo.class, "rotate");
        grabberLeft = hardwareMap.get(Servo.class, "grabberLeft");
        grabberRight = hardwareMap.get(Servo.class, "grabberRight");
    
        grabberLeft.setPosition(0.74);
        grabberRight.setPosition(0.14);
        
        
        m1 = hardwareMap.get(DcMotorEx.class, "frontl");
        m2 = hardwareMap.get(DcMotorEx.class, "backl");
        m3 = hardwareMap.get(DcMotorEx.class, "frontr");
        m4 = hardwareMap.get(DcMotorEx.class, "backr");
        
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        //m3.setDirection(DcMotor.Direction.REVERSE);
        //m4.setDirection(DcMotor.Direction.REVERSE);
        
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        
        
        
        rotate.setPosition(0.85);
        
        waitForStart();
        
        drive(1,0,0);
        sleep(2600);
        stopDrive();
        
        //turn 90 degrees left
        drive(0,0,-0.5);
        sleep(2035);
        stopDrive();
        
        //turn 90 degrees right
        sleep(2000);
        drive(0,0,0.5);
        sleep(2035);
        stopDrive();
        
        //turn 180 degrees
        sleep(2000);
        drive(0,0,0.5);
        sleep(4070);
        stopDrive();
        
        //turn 90 degrees left
        sleep(2000);
        drive(0,0,-0.5);
        sleep(2035);
        stopDrive();
        
    }
}
