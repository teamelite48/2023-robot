package frc.robot.subsystems.drive;

public enum PathType {

    Test("Test Path"),
    UTurn("U Turn Path"),
    UTurnCopy("U Turn Path Copy");

    public final String pathName;

    private PathType(String pathName) {
        this.pathName = pathName;
    }
}
