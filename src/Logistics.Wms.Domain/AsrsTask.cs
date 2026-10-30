namespace Logistics.Wms.Domain;

/// <summary>A transport order for R1 (IRB 6700 with L-fork) inside the cylinder shelf.</summary>
public class AsrsTask
{
    public int Id { get; set; }
    public AsrsTaskKind Kind { get; set; }
    public AsrsTaskStatus Status { get; set; }
    public required string BinId { get; set; }
    public int? FromSlotId { get; set; }
    public int? ToSlotId { get; set; }
    /// <summary>SKU demand that triggered a retrieval.</summary>
    public int? ForSkuId { get; set; }
    public DateTime CreatedAt { get; set; }
    public DateTime? CompletedAt { get; set; }
}
