namespace Logistics.Wms.Domain;

public enum RobotRole
{
    /// <summary>R1 (IRB 6700, L-fork): bins between the cylinder shelf and the loops.</summary>
    ShelfRobot,
    /// <summary>R2 / R3 (IRB 1300, 4-cup vacuum, 3D vision): cartons from the bin at a picking stop onto the item belt.</summary>
    Picker,
    /// <summary>S1 / S2 (IRB 930 SCARA, bellows cup): cartons from the belt stop into the order cases of four put-wall lanes.</summary>
    Sorter,
}

/// <summary>Operating mode, set by people.</summary>
public enum RobotMode
{
    /// <summary>The WMS dispatches work to the robot.</summary>
    Auto,
    /// <summary>A technician jogs or teaches the robot; the WMS gives it no work.</summary>
    Manual,
    /// <summary>Out of service for maintenance; the WMS gives it no work.</summary>
    Maintenance,
}

/// <summary>Run state, reported by the robot controller (or the WMS's own task tracking).</summary>
public enum RobotState { Idle, Working, Fault }

public enum RobotEventKind { Fault, FaultCleared, ModeChanged, WorkReassigned, StatsReset }

/// <summary>One of the five robots of the cell, with its current task and lifetime figures.</summary>
public class Robot
{
    /// <summary>"R1", "R2", "R3", "S1", "S2".</summary>
    public required string Id { get; set; }
    public required string Model { get; set; }
    public RobotRole Role { get; set; }
    /// <summary>Picking stop served by a picker.</summary>
    public PickStation? Station { get; set; }
    /// <summary>Sorter position (lanes S1-1…4 / S2-1…4) of a sorter.</summary>
    public Sorter? Sorter { get; set; }
    public required string Tool { get; set; }
    public required string Controller { get; set; }

    public RobotMode Mode { get; set; }
    public RobotState State { get; set; }
    public string? FaultCode { get; set; }
    public string? FaultText { get; set; }
    public DateTime? FaultSince { get; set; }

    /// <summary>Reference of the task the robot works on ("asrs:12", "pick:345", "sort:345").</summary>
    public string? TaskRef { get; set; }
    public string? TaskText { get; set; }
    public DateTime? TaskStartedAt { get; set; }

    public int Cycles { get; set; }
    public double BusySeconds { get; set; }
    public double LastCycleSeconds { get; set; }
    public double FaultSeconds { get; set; }
    public int Faults { get; set; }
    /// <summary>Start of the figures (utilisation, availability are relative to it).</summary>
    public DateTime StatsSince { get; set; }

    /// <summary>The WMS may give the robot work.</summary>
    public bool IsAvailable => Mode == RobotMode.Auto && State != RobotState.Fault;
}

public class RobotEvent
{
    public long Id { get; set; }
    public required string RobotId { get; set; }
    public DateTime At { get; set; }
    public RobotEventKind Kind { get; set; }
    public string? Code { get; set; }
    public required string Text { get; set; }
    public string? User { get; set; }
}
