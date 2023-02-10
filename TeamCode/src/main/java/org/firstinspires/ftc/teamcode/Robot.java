package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;

@Config
public class Robot {

   public  DcMotorEx hombro;
  // public DcMotor hombro;
   public DcMotor brazo;
   public DcMotor elev;
   public DcMotor elev2;
   public Servo garra1;
   public Servo garra2;

   public static PIDCoefficients rielesPID = new PIDCoefficients(0.0011, 0, 0);

   public PIDFController rielesController = new PIDFController(rielesPID);

   public SampleMecanumDrive drive;
   public OpenCvCamera camara;


   public void init(HardwareMap hardwareMap) {

        brazo = hardwareMap.dcMotor.get("brazo");//motor expansion 3
        elev = hardwareMap.dcMotor.get("elev");//motor expansion 0
        elev2 = hardwareMap.dcMotor.get("elev2");//motor expansion 1
        garra1 = hardwareMap.servo.get("garra1");//servo control 0
      //  hombro = hardwareMap.dcMotor.get("hombro");
        drive = new SampleMecanumDrive(hardwareMap);
        garra2 = hardwareMap.servo.get("garra2");
        hombro = hardwareMap.get(DcMotorEx.class,"hombro");





      brazo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      brazo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       hombro.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       hombro.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      // hombro.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       //hombro.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       hombro.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       brazo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       elev.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       elev2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       elev.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       elev2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void elevador(double power) {


        elev.setPower(power);


        elev2.setPower(power);

    }


    public void brazoauto(double power,int ticks){
       brazo.setTargetPosition(ticks);
       brazo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       brazo.setPower(power);
    }

   public void brazito(double power){

        brazo.setPower(power);

   }

   public void garra1(double position){
       garra1.setPosition(position);
   }

    public void garra2(double position){
         garra2.setPosition(position);
    }

    public void elevadores(double power, int ticks){
       elev.setTargetPosition(ticks);
       elev.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       elev.setPower(power);

        elev2.setTargetPosition(ticks);
        elev2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elev2.setPower(power);
    }

    public void elevadorAuto(double power, int ticks) {
       rielesController.setTargetPosition(ticks);
       rielesController.setOutputBounds(-1, 1);

       double rielesPower = rielesController.update(elev.getCurrentPosition()) * power;

       elev.setPower(rielesPower);
       elev2.setPower(rielesPower);
    }

    public void hombrito(double power){
       hombro.setPower(power);

    }

   public void hombritoAuto (double power, int posicion){
       hombro.setTargetPosition(posicion);
       hombro.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       hombro.setPower(power);
    }

}
