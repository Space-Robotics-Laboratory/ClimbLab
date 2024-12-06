# Path Planning

```mermaid
classDiagram

  class ConfigPathPlanning {
    + goal_position: 3x1 double
    + global_path_plan_type: string
    + local_path_plan_type: string

    + override()
    + isValidPathPlanningMethod()
  }

  class PathPlanning {
    + global_path: GlobalPathPlanning
    + local_path: LocalPathPlanning
  }
  PathPlanning *-- GlobalPathPlanning
  PathPlanning *--  LocalPathPlanning

  class GlobalPathPlanning {
    + type: string
    + planner: GlobalPathPlanner
    + goal_position: 3x1 double
    + path: Trajectory

    + GlobalPathPlanning()
    + plan()
    + setPlanner()
    + getGoalPosition()
    + getGlobalPath()
  }
  GlobalPathPlanning *-- GlobalPathPlanner

  class GlobalPathPlanner {
    <<Abstract>>
  }
  GlobalPathPlanner <|-- StraightTowardGoalDirection
  GlobalPathPlanner <|-- DynamicGlobal
  GlobalPathPlanner <|-- GraphBased

  class StraightTowardGoalDirection {
    + plan()
  }

  class DynamicGlobal {
    <<Haji_algo>>
    + plan()
  }

  class GraphBased {
    <<Takady_algo>>
    + plan()
  }

  class LocalPathPlanning {
    + type: string
    + planner: LocalPathPlanner
    + moving_direction: 3x1 double

    + LocalPathPlanning()
    + plan()
    + setPlanner()
    + getMovingDirection()
  }
  LocalPathPlanning *-- LocalPathPlanner

  class LocalPathPlanner {
    <<Abstract>>
  }
  LocalPathPlanner <|-- LPPBasedOnNextWayPoint

  class LPPBasedOnNextWayPoint {
    + plan()
  }


```
