```mermaid
flowchart TD
    Prepare_Handoff --> Prepare_Handoff_Left
    Prepare_Handoff --> Prepare_Handoff_Right
    Prepare_Handoff --> Prepare_Handoff_Middle
    Prepare_Handoff_Coral_Mode --> Prepare_Handoff_Left
    Prepare_Handoff_Coral_Mode --> Prepare_Handoff_Right
    Prepare_Handoff_Coral_Mode --> Prepare_Handoff_Middle
    subgraph CORAL
        Prepare_Handoff_Left
        Prepare_Handoff_Right
        Prepare_Handoff_Middle
    end
```