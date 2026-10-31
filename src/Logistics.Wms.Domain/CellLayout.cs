namespace Logistics.Wms.Domain;

/// <summary>
/// Fixed layout of the robot cell (see canvas/project/app.js of the 3D simulation): cylinder shelf of 26 columns × 8 levels with
/// eight columns left out for the ports and the maintenance passage (144 slots), two picking stops, two sorters with four put-wall
/// lanes each, a picking loop and a restock loop with limited buffer places, and an exception tote at the belt end.
/// </summary>
public sealed record RobotDefinition(string Id, string Model, RobotRole Role, PickStation? Station, Sorter? Sorter, string Tool, string Controller);

public static class CellLayout
{
    public const int Columns = 26;
    public const int Levels = 8;
    /// <summary>East port (25, 0, 1) picking loop, west port (12, 13, 14) restock loop, north passage (6, 7).</summary>
    public static readonly IReadOnlySet<int> GapColumns = new HashSet<int> { 25, 0, 1, 12, 13, 14, 6, 7 };
    public const double RingLevel0Height = 0.30;
    public const double RingLevelPitch = 0.37;
    /// <summary>Preferred working height of R1; slotting prefers slots near it.</summary>
    public const double PreferredHeight = 1.4;
    public const double HeightWeight = 0.35;

    /// <summary>Bins on the picking loop (inbound, at the stops, return) before R1 stops retrieving.</summary>
    public const int PickLoopCapacity = 7;
    /// <summary>Bins on the restock loop before R1 stops sending empty bins from the return head.</summary>
    public const int RestockLoopCapacity = 4;
    /// <summary>Lower limit for sending empty bins from the rack, so a returning bin always finds a place.</summary>
    public const int RestockLoopRackLimit = 3;
    public const int ExceptionToteCapacity = 25;

    public static readonly IReadOnlyList<(Sorter Sorter, int Index)> Lanes =
        [.. Enum.GetValues<Sorter>().SelectMany(s => Enumerable.Range(1, 4).Select(i => (s, i)))];

    public static string LaneId(Sorter sorter, int index) => $"{sorter}-{index}";

    public static Sorter LaneSorter(string laneId) => laneId.StartsWith("S2", StringComparison.OrdinalIgnoreCase) ? Domain.Sorter.S2 : Domain.Sorter.S1;

    public static IEnumerable<string> LaneIdsOf(Sorter sorter) => Lanes.Where(l => l.Sorter == sorter).Select(l => LaneId(l.Sorter, l.Index));

    /// <summary>The five robots (models and tools as in the cell simulation).</summary>
    public static readonly IReadOnlyList<RobotDefinition> Robots =
    [
        new("R1", "IRB 6700-150/3.20", RobotRole.ShelfRobot, null, null, "L-fork, bins ≤ 20 kg", "OmniCore V250XT"),
        new("R2", "IRB 1300-12/1.4", RobotRole.Picker, PickStation.P1, null, "4-cup vacuum gripper, 3D camera", "OmniCore C30"),
        new("R3", "IRB 1300-12/1.4", RobotRole.Picker, PickStation.P2, null, "4-cup vacuum gripper, 3D camera", "OmniCore C30"),
        new("S1", "IRB 930-22/1.05", RobotRole.Sorter, null, Domain.Sorter.S1, "bellows cup", "OmniCore C30"),
        new("S2", "IRB 930-22/1.05", RobotRole.Sorter, null, Domain.Sorter.S2, "bellows cup", "OmniCore C30"),
    ];

    public const string ShelfRobotId = "R1";

    /// <summary>Picker serving a stop: R2 at P1, R3 at P2.</summary>
    public static string PickerOf(PickStation station) => station == PickStation.P1 ? "R2" : "R3";

    public static string SorterRobotOf(Sorter sorter) => sorter.ToString();

    /// <summary>Cartons per bin: two layers of a grid (S 4 × 4, M 3 × 3, L 2 × 2).</summary>
    public static int BinCapacity(SkuSize size) => size switch
    {
        SkuSize.S => 32,
        SkuSize.M => 18,
        SkuSize.L => 8,
        _ => throw new ArgumentOutOfRangeException(nameof(size)),
    };

    public static double ColumnAngle(int column) => WrapAngle(column * 2 * Math.PI / Columns);

    public static double LevelHeight(int level) => RingLevel0Height + level * RingLevelPitch;

    /// <summary>Angle normalised to (−π, π].</summary>
    public static double WrapAngle(double a) => Math.Atan2(Math.Sin(a), Math.Cos(a));
}
