namespace Logistics.Wms.Domain;

/// <summary>A powered order-case lane of the put wall (one order per lane, rolling waves).</summary>
public class PutWallLane
{
    /// <summary>"S1-1" … "S2-4".</summary>
    public required string Id { get; set; }
    public Sorter Sorter { get; set; }
    public int Index { get; set; }
    public LaneState State { get; set; }
    public int? OrderId { get; set; }
    /// <summary>Lane switched off by an operator: it finishes its order but takes no new one.</summary>
    public bool Disabled { get; set; }
    public int CasesCompleted { get; set; }
    public DateTime? StateSince { get; set; }
}
