namespace Logistics.Wms.Domain;

/// <summary>
/// One carton for one order line: allocated in a bin at a picking stop, picked onto the belt by R2 / R3, scanned at the
/// bar-code arch and put into the order case by S1 / S2. Its id is the tracking number of the carton on the belt.
/// </summary>
public class PickTask
{
    public long Id { get; set; }
    public required string BinId { get; set; }
    public PickStation Station { get; set; }
    public int OrderId { get; set; }
    public int OrderLineId { get; set; }
    public OrderLine OrderLine { get; set; } = null!;
    public int SkuId { get; set; }
    /// <summary>Position in the bin's pick sequence (top layer first).</summary>
    public int Sequence { get; set; }
    public PickStatus Status { get; set; }
    public Sorter? TargetSorter { get; set; }
    public string? TargetLaneId { get; set; }
    public DateTime CreatedAt { get; set; }
    public DateTime? PickedAt { get; set; }
    public DateTime? ScannedAt { get; set; }
    public DateTime? SortedAt { get; set; }
}
