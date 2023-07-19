package org.team498.C2023;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotState extends SubsystemBase {
    private GameMode currentGameMode = GameMode.CONE;
    private ScoringOption nextScoringOption = ScoringOption.MID;
    private State currentState = State.IDLE_CUBE;
    private boolean shootDrive = false;

    public enum GameMode {CUBE, CONE}
    public enum ScoringOption {TOP, MID, LOW, SPIT}


    public void setCurrentGameMode(GameMode gameMode) {currentGameMode = gameMode; Logger.getInstance().recordOutput("Game Mode", gameMode.name());}
    public boolean inCubeMode() {return currentGameMode == GameMode.CUBE;}
    public boolean inConeMode() {return currentGameMode == GameMode.CONE;}

    public void setState(State state) {currentState = state; Logger.getInstance().recordOutput("Robot State", state.name());}
    public State getState() {return currentState;}

    public void setNextScoringOption(ScoringOption scoringOption) {nextScoringOption = scoringOption; Logger.getInstance().recordOutput("Scoring Option", scoringOption.name());}
    public ScoringOption getNextScoringOption() {return nextScoringOption;}

    public void setShootDrive(boolean enable) {shootDrive = enable;}
    public boolean inShootDriveMode() {return shootDrive;}

    public State getNextScoringState() {
        State state;

        if (shootDrive) {
            state = switch (nextScoringOption) {
                case TOP -> State.SHOOT_DRIVE_CUBE_TOP;
                case MID -> currentGameMode == GameMode.CONE ? State.SHOOT_DRIVE_CONE_MID : State.SHOOT_DRIVE_CUBE_MID;
                case SPIT -> State.SPIT_CUBE;
                case LOW -> State.LOW_CONE;
            };
        } else {
            state = switch (nextScoringOption) {
                case TOP -> currentGameMode == GameMode.CONE ? State.TOP_CONE : State.TOP_CUBE;
                case MID -> currentGameMode == GameMode.CONE ? State.MID_CONE : State.MID_CUBE;
                case SPIT -> State.SPIT_CUBE;
                case LOW -> State.LOW_CONE;
            };
        }

        return state;
    }

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }
}
