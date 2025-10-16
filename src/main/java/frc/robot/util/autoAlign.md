```mermaid
flowchart TD
    subgraph IN["Inputs"]
        RPose("Robot Pose")
        AllianceReef("Alliance Reef Pose")
        cameraConnection("Camera Connection Status")
        scoringLevel("Scoring Level")
    end
    subgraph PROCESS["Process"]
        getClosestReef{Get Closest Reef}
            RPose --> getClosestReef
            AllianceReef --> getClosestReef
        getClosestBranch{Get Closest Branch}
            RPose --> getClosestBranch
            getClosestReef --> getClosestBranch
        distanceToBranch{Distance To Branch}
            RPose --> distanceToBranch
            getClosestBranch --> distanceToBranch
                leftOrRight{Left Or Right Preference}
                RPose --> leftOrRight
                cameraConnection --> leftOrRight
                AllianceReef --> leftOrRight
    end
    subgraph OUT["Outputs"]
        ChassisSpeeds("Chassis Speeds")
        IsAligned("Is Aligned")

    end
    IN --> PROCESS
    PROCESS --> OUT
    ```
