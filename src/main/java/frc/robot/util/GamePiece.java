// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class GamePiece {
    public static GamePieceType gamePiece;
    public static enum GamePieceType{
        Cone, Cube, None;
    }

    public static void setGamePiece(GamePieceType gamePieceType){
        gamePiece = gamePieceType;
    }

    public static GamePieceType getGamePiece(){
        return gamePiece;
    }
}
