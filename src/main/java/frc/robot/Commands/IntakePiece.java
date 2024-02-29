package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShintakeSubsystem;


public class IntakePiece extends Command{
    private ShintakeSubsystem shintake;
    private double intakespeed;
    private boolean pieceReady;
    private boolean beam0Empty;
    private boolean beam1Empty;
    private boolean beam2Empty;
    

    public IntakePiece(ShintakeSubsystem shintake, double intakespeed) {
        addRequirements(shintake);
        this.intakespeed = intakespeed;
        this.shintake = shintake;
    }
    
    public void initialize() {
        this.beam0Empty = shintake.beam0Empty.get();
        this.beam1Empty = shintake.beam1Empty.get();
        this.beam2Empty = shintake.beam2Empty.get();
    }


    @Override
    public void execute() {
        if (beam0Empty && beam1Empty && beam2Empty) { //everything is empty the world is sad. FILL ME WITH NOTES OF JOY!
            shintake.setIntakeSpeed(intakespeed);
            pieceReady = false;

        } else if (!beam0Empty && !beam1Empty && beam2Empty) { //a note exists but does not touch shooter (THIS IS VERY GOOD)
            shintake.setIntakeSpeed(0);
            pieceReady = true;

        } else if (!beam1Empty && !beam2Empty){ //a note exists but goes too far and touches the shooter (THIS IS VERY BAD)
            shintake.setIntakeSpeed(-intakespeed);
            pieceReady = false;
        }
    }
}
