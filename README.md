# 2023-robot

## Absolute Encoder Position Conversion Factor
- J1 Shoulder: 20:40
- J2 Elbow: 1:1
- J3 Wrist: 15:52

## Relative Encoder Position Conversion Factor
- J1 Shoulder: 485.33:1
- J2 Elbow: 213.89:1
- J3 Reduction: 30:1 gives you 360/(42*30) = 0.2857 degrees per tick.



Feature: Better Arm Motions

    Scenario: Arm Deploys to Low Drop Zone Before Commanded Position

        Given the arm is inside the frame perimeter
        When the arm is commanded to move outside of the frame perimeter and bellow the mid drop zone
        Then the arm moves to the low drop zone before the commanded position

    Scenario: Arm Deploys to High Drop Zone Before Commanded Position

        Given the arm is inside the frame perimeter
        When the arm is commanded to move outside the frame perimeter and above the high drop zone
        Then then arm moves to the high drop zone before the commanded position

    Scenario: Arm Retracts to Low Drop Zone Before Commanded Position

        Given the arm is outside the frame perimeter
        When the arm is commanded to move inside of the frame perimeter and is bellow the high drop zone
        Then the arm moves to low drop zone before the commanded position

    Scenario: Arm Retracts to High Drop Zone Before Commanded Position

        Given the arm is outside the frame perimeter
        When the arm is commanded to move inside of the frame perimeter and is above the high drop zone
        Then the arm moves to the high drop zone before the commanded position

    Scenario: Arm Moves to Commanded Position Within the Frame Perimeter

        Given the arm is inside of the the frame perimeter
        When the arm is commanded to move inside of the frame perimeter
        Then the arm moves to the commanded position without deploying to either the low or high drop zones first

Ready Arm
|
|-> Ready Arm from Inside Frame Perimeter
|   |
|   |-> Commanded Inside
|   |       * Move to Commanded Position
|   |
|   |-> Commanded Outside
|       |
|       |-> Commanded Bellow Low Drop Zone
|       |       * Move to Low Drop Zone
|       |       * Move to Commanded Position
|       |
|       |-> Commanded Above High Drop Zone
|               * Move to High Drop Zone
|               * Move to Commanded Position
|
|
|-> Ready Arm from Outside Frame Perimeter
    |
    |-> Commanded Inside
    |   |
    |   |-> Currently Bellow High Drop Zone
    |   |       * Move to Low Drop Zone
    |   |       * Move to Commanded Position
    |   |-> Currently Above High Drop Zone
    |           * Move to High Drop Zone
    |           * Move to Commanded Position
    |
    |-> Commanded Outside
        |
        |-> Move to Commanded Position

