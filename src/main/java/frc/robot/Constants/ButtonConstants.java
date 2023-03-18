package frc.robot.Constants;

public class ButtonConstants {

  public static final int CONTROLLER_PORT = 0;
  public static final int BUTTON_PANEL_PORT = 1;
  /*
   * Button Panel Mapping
   * ---------------------------
   * |          | 11 | 12|     | 
   * |    ^    | 1 | 2 | 3 | 4 |
   * |   < >   | 5 | 6 | 7 | 8 |
   * |    v    | 9 | 10 |      |   
   * ---------------------------
   */

  // All Constants are subject to change once we figure out what each one does in the old code.
  //12 is not connected so it is 1-11 
  /* Actual Buttons */
  public static final int HighCone = 1;
  public static final int HighCube = 2;

  public static final int ClawIntake = 3;
  public static final int ClawOuttake = 4;

  public static final int TurretLeft = 5;
  public static final int TurretRight = 6;

  public static final int MidCone = 5;
  public static final int MidCube = 6;
  

  public static final int Ground = 9;
  public static final int CLAW_TOGGLE = 10;

  public static final int ResetEncoder = 11;
  /* Actual Buttons */

  /* Test Buttons */
  public static final int clawToggle = 1;
  public static final int armToggle = 2;
  public static final int vfbarDown = 3;
  public static final int vfbarUp = 4;
  public static final int clawIn = 5;
  public static final int clawOut = 6;
  /* Test Buttons */


   
}