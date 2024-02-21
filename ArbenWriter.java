package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.text.SimpleDateFormat;
import java.text.DateFormat;
import java.util.Date;
import java.io.FileWriter;
import java.io.BufferedWriter;
import java.io.IOException;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="ArbenWriter")
public class ArbenWriter extends LinearOpMode {

    private Servo rotate;
    private Servo grabberLeft;
    private Servo grabberRight;
    private Servo droneLauncher;
    
    DcMotorEx m1, m2, m3, m4;
    
    DcMotor armRotatorLeft, armRotatorRight;
    
    private boolean leftClawMotor = false;
    private boolean rightClawMotor = false;
    
    private boolean rotateIsRotated = true;
    
    private long lastPressed = 0;
    BufferedWriter out = null;
    
    DateFormat dateFormat = new SimpleDateFormat("MM/dd 'at' HH:mm:ss");
    Date date = new Date();
    
    public void writeInstruction(String text){
        try {
            out.write(text);
            out.write("\n");
            out.flush();
        } catch (IOException r){
            r.printStackTrace();
        }
    }


    @Override
    public void runOpMode(){
        try {
            FileWriter fstream = new FileWriter("sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/Instructions.txt", true);
            out = new BufferedWriter(fstream);
            out.write("\n----------NEW INSTRUCTIONS " + dateFormat.format(date) + "----------\n");
        } catch(IOException e) {
            e.printStackTrace();
        }

         //Grabber motors
        rotate = hardwareMap.get(Servo.class, "rotate");
        grabberLeft = hardwareMap.get(Servo.class, "grabberLeft");
        grabberRight = hardwareMap.get(Servo.class, "grabberRight");
        droneLauncher = hardwareMap.get(Servo.class, "droneLauncher");
        
        // Wheels, ordered accordingly:
        // Top Left, Back Left, Front Right, Back Right
        m1 = hardwareMap.get(DcMotorEx.class, "frontl");
        m2 = hardwareMap.get(DcMotorEx.class, "backl");
        m3 = hardwareMap.get(DcMotorEx.class, "frontr");
        m4 = hardwareMap.get(DcMotorEx.class, "backr");
        
        //Arm Rotators
        armRotatorLeft = hardwareMap.get(DcMotor.class, "armRotatorLeft");
        armRotatorRight = hardwareMap.get(DcMotor.class, "armRotatorRight");
        
        armRotatorLeft.setDirection(DcMotor.Direction.REVERSE);
        
        //Motor direction, flip to reverse direction of robot (may need to reorder motors)
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        //m3.setDirection(DcMotor.Direction.REVERSE);
        //m4.setDirection(DcMotor.Direction.REVERSE);
        
        //not going to lie i have no idea what this stuff is 
        // m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        armRotatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        telemetry.addData("Press Start When Ready","");
        telemetry.update();
        
        //  UNCOMMENT TO ENABLE MOVE ON INITIALIZATION
        
        //Left - 0-1 Closed-Opened
        //Right - 0-1 Opened-Closed
        
        // Close left-0.74
        // CloseRight-0.14
        
        grabberLeft.setPosition(0.74);
        grabberRight.setPosition(0.14);
        
        droneLauncher.setPosition(0);

        waitForStart();
        
        //rotate.setPosition(1);
        
        boolean isGrabbed = false;

        while (opModeIsActive()) {

                // BODY MOVEMENT
            double px = gamepad1.left_stick_x; //The power of the left stick on the x axis
                                                // Left to Right, -1.00 to 1.00
            double py = -gamepad1.left_stick_y; //The power of the left stick on the y axis
                                                // Down to Up, -1.00 to 1.00
            double pa = gamepad1.left_trigger - gamepad1.right_trigger;
                                                //The power of the body rotation

            //math equation stuff
            if (Math.abs(pa) < 0.05) pa = 0;
            //p1 correlates to m1
            //p2 correlates to m2
            // etc.
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

            //Positions of the motors
            telemetry.addData("Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(),
                    m3.getCurrentPosition(), m4.getCurrentPosition());
            telemetry.update();
            
                //GRABBER MOVEMENT
            
            
            //Left - 0-1 Closed-Opened
            //Right - 0-1 Opened-Closed
            
            /*
            Y- Open/Close Right
            X- Open/Close Left
            
            Close left-0.74
            CloseRight-0.14
            
            OpenLeft-1
            OpenRight-0
            
            
            */
            if(gamepad2.x){ //Open/Close Left
                if((grabberLeft.getPosition() == 1) && (System.currentTimeMillis() - lastPressed > 250)){ //Checks if open, close
                    grabberLeft.setPosition(0.74);
                    lastPressed = System.currentTimeMillis();
                    
                } else if ((grabberLeft.getPosition() == 0.74) && (System.currentTimeMillis() - lastPressed > 250)){ //Checks if closed, open
                    grabberLeft.setPosition(1);
                    lastPressed = System.currentTimeMillis();
                }
            }
            
            if(gamepad2.y){ //Open/Close Right
                if((grabberRight.getPosition() == 0) && (System.currentTimeMillis() - lastPressed > 250)){ //Checks if open, close
                    grabberRight.setPosition(0.14);
                    lastPressed = System.currentTimeMillis();
                } else if ((grabberRight.getPosition() == 0.14) && (System.currentTimeMillis() - lastPressed > 250)){ //Checks if closed, open
                    grabberRight.setPosition(0);
                    lastPressed = System.currentTimeMillis();
                }
            }
            
            if(gamepad2.right_bumper && rotateIsRotated){ //Down
                rotateIsRotated = false;
                rotate.setPosition(0.67);
                writeInstruction("rd");
            }
            if(gamepad2.left_bumper && rotateIsRotated == false){ //Up
                rotateIsRotated = true;
                rotate.setPosition(1);
                writeInstruction("ru");
            }
            
            double ap = gamepad2.right_stick_y;
            
            if(ap > 0 || ap < 0){
                armRotatorLeft.setPower(ap);
                armRotatorRight.setPower(ap);
            } else {
                armRotatorLeft.setPower(0);
                armRotatorRight.setPower(0);
            }
            
            //drone launcher
            
            if(gamepad1.guide){
                droneLauncher.setPosition(1);
            }
            
            
        }
        //Stops all motors
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        
        armRotatorLeft.setPower(0);
        armRotatorRight.setPower(0);
        
        try {
            out.close();
        } catch (IOException u){
            u.printStackTrace();
        }
    }
}
