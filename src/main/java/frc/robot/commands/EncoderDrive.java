package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class EncoderDrive extends Command {
    Drivetrain dt;
    double setPoint;

    public EncoderDrive(Drivetrain dt, double setPoint){
        this.dt = dt;
        this.setPoint = setPoint;
        addRequirements(dt);
    }
    @Override
    public void initialize(){
        dt.resetEncoders();
        dt.tankDrive(0,0); 
    }

    @Override
    public void execute(){
        dt.tankDrive(0.5,0.5);
    }

    @Override
    public void end(boolean interrupted){
        dt.tankDrive(0,0);
    }

    @Override
    public boolean isFinished(){
        //this will return true when the command has finished
        if (dt.getMeters() >= setPoint){
            return true;
        }
        else{
            return false;
        }
    }
}