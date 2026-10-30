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
}
