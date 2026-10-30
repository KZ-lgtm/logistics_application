namespace Logistics.Wms.Domain;

/// <summary>Single-row state of the cell that the WMS needs for its decisions.</summary>
public class CellState
{
    public const int SingletonId = 1;

    public int Id { get; set; } = SingletonId;
    /// <summary>Last known column angle of R1 (rad); slotting and bin choice minimise the swing from here.</summary>
    public double R1Angle { get; set; }
    /// <summary>Cartons physically in the exception tote (reset when the operator empties it).</summary>
    public int ExceptionToteCount { get; set; }

    // totals of the history removed by compaction, so lifetime KPIs survive it
    public int ArchivedOrdersCompleted { get; set; }
    public int ArchivedOrdersCancelled { get; set; }
    public int ArchivedItemsPicked { get; set; }
    public int ArchivedItemsSorted { get; set; }
    public int ArchivedScans { get; set; }
    public int ArchivedNoReads { get; set; }
}
