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

public class ArbenAutoRedRight extends LinearOpMode {
    
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
        
        
        
        
        waitForStart();
        
        rotate.setPosition(0.67);

        drive(0.35,0,0);
        while(hasSeenLine == false){
            int red = sensorColor.red();

            telemetry.addData("Red  ", red);
            telemetry.update();
            if(red > 200){
                stopDrive();
                hasSeenLine = true;
            }
        }
        
        drive(-0.3,0,0);
        sleep(400);
        stopDrive();
        
        if(DSL.getDistance(DistanceUnit.CM) <= 15){ //Checks if on left
        
            //placing pixel on line
            drive(-0.5,0,0);
            sleep(750);
            stopDrive();
            
            drive(0,0,-0.5);
            sleep(1350);
            stopDrive();
            grabberLeft.setPosition(1);
            sleep(500);
            rotate.setPosition(1);
            
            //Backing up out of the way
            sleep(300);
            drive(-0.5,0,0);
            sleep(900);
            stopDrive();
            rotate.setPosition(0.67);
            sleep(500);
            
            //placing pixel on board
            drive(0,0,-0.5);
            sleep(300);
            stopDrive();
            
            
            
        } else if(DSR.getDistance(DistanceUnit.CM) <= 15){// Checks if on right
            drive(-0.5,0,0);
            sleep(750);
            stopDrive();
            
            drive(0,0,0.5);
            sleep(1600);
            stopDrive();
            drive(0.2,0,0);
            sleep(600);
            stopDrive();
            grabberLeft.setPosition(1);
            sleep(500);
            rotate.setPosition(1);
            
        } else { //If neither, must be in center
            drive(-0.7,0,0);
            sleep(400);
            stopDrive();
            grabberLeft.setPosition(1);
            sleep(500);
            rotate.setPosition(1);
            
        }
        
        // sleep(300);
        // drive(-0.5,0,0);
        // sleep(670);
        // stopDrive();
        // rotate.setPosition(0.67);
        // sleep(500);
        
    }
}
