# O2025 - Developer Guide

**FRC Team 498 - The Cobra Commanders**  2025 Offseason C Robot Code

## Quick Start for New Developers

### 1. Development Environment Setup

**Install VS Code with WPILib:**

Follow the official guide: [WPILib Installation](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)

**Clone and Setup:**

#### Github Desktop

TODO

#### Git in Command Line

```bash
git clone https://github.com/cobracommanders/O2025.git
cd O2025
code .  # Opens in VS Code
```

**Build and Deploy:**

- Build: `Ctrl+Shift+P` → "WPILib: Build Robot Code"- Deploy: `Ctrl+Shift+P` → "WPILib: Deploy Robot Code"

### 2. Project Structure

```
src/main/java/frc/robot/
├── Robot.java # Main robot class
├── RobotContainer.java # Command binding
├── Constants.java # Robot constants
├── commands/ # Robot commands
├── subsystems/ # Robot subsystems
└── util/ # Utility classes
```

## Java Best Practices

### Example Naming Conventions

```java
// Classes: PascalCase
public class DriveSubsystem extends SubsystemBase { }

// Methods and variables: camelCase
private double maxSpeed;public void setMotorPower(double power) { }

// Constants: SCREAMING_SNAKE_CASE
public static final double MAX_SPEED_MPS = 4.5;

// Packages: lowercasepackage
frc.robot.subsystems;
```

### Code Organization

```java
// Good: Clear, descriptive names
public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor;
    private final DigitalInput beamBreak;
    public void runIntake(double speed) {
        intakeMotor.set(speed);    
        }        
    public boolean hasGamePiece() {
        return !beamBreak.get(); // Beam break is normally closed    
    }
}

// Bad: Unclear names, no comments
public class Thing extends SubsystemBase {
    private final CANSparkMax m1;
    private final DigitalInput d1; 
    public void go(double s) { 
        m1.set(s);    
    }
}
```

### Command-Based Programming

```java
// Commands should be specific and reusable
public class IntakeGamePiece extends CommandBase {    
    private final IntakeSubsystem intake;

    public IntakeGamePiece(IntakeSubsystem intake) {        
        this.intake = intake;
        addRequirements(intake);
    }
        
    @Override    
    public void execute() {
        intake.runIntake(0.8);
    }
    
    @Override    
    public boolean isFinished() {
        return intake.hasGamePiece();    
    }
    
    @Override    
    public void end(boolean interrupted) {
        intake.runIntake(0);    
    }
}
```

## Git Workflow

### Branch Naming Convention

```
main # Stable, competition-ready code
event-<event-name>  # Event specific code, updates or release
<feature-description> # New features
fix-<bugfix-description> # Bug fixes
```

### Workflow Process

**1. Starting New Work:**

```bash
git checkout main
git pull origin main
git checkout -b your-feature-name
```

**2. Making Changes:**

```bash
# Make your changes
git add .git commit -m "Add swerve drive kinematics calculation"
```

**4. Before Pull Request (PR):**

```bash
# Update with latest main (via pull)
git pull origin main
# resolve merge conflicts

# Update with latest main (via rebase)
git checkout main
git pull origin main
git checkout you-feature-name
git rebase main
# resolve rebase issues
```

### Pull Request Guidelines

**Before Creating PR:**

- [ ] Code builds without errors
- [ ] Code tested on robot (if hardware-related)
- [ ] No merge conflicts with develop
- [ ] Code follows style guidelines

## Development Tips for New Programmers

### Understanding Robot Code Flow1

**Robot.java** - Entry point, calls periodic methods2. **RobotContainer.java** - Sets up subsystems and binds controls3. **Subsystems** - Control hardware (motors, sensors)4. **Commands** - Define robot behaviors using subsystems

### Common Pitfalls to Avoid

```java
// DON'T: Create objects in periodic methods
public void teleopPeriodic() {    
    Command cmd = new DriveCommand(); // Creates new object every 20ms!
}
// DO: Create once, reuse
private final Command driveCommand = new DriveCommand();
```

### Debugging Strategies

```java
// TODO: doglog
```

## Useful Resources

- [WPILib Documentation](https://docs.wpilib.org/)
- [Chief Delphi](https://www.chiefdelphi.com/)
- [Java Documentation](https://docs.oracle.com/en/java/)
- [Git Tutorial](https://git-scm.com/docs/gittutorial)
