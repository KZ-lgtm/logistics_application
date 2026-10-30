namespace Logistics.Wms.Domain;

/// <summary>A bar-coded storage bin holding cartons of one SKU.</summary>
public class Bin
{
    /// <summary>Bin bar code (e.g. "B001").</summary>
    public required string Id { get; set; }
    public int SkuId { get; set; }
    public Sku Sku { get; set; } = null!;
    public int Quantity { get; set; }
    public BinLocation Location { get; set; }
    public int? SlotId { get; set; }
    public StorageSlot? Slot { get; set; }
    /// <summary>Picking stop the bin is routed to / standing at.</summary>
    public PickStation? Station { get; set; }
    /// <summary>True once picks were allocated for this presentation (at the stop).</summary>
    public bool Allocated { get; set; }
    /// <summary>An ASRS task has been assigned for this bin.</summary>
    public bool Reserved { get; set; }
    public DateTime? LastMovedAt { get; set; }

    public bool IsEmpty => Quantity <= 0;
    public bool IsOnPickLoop => Location is BinLocation.PickInbound or BinLocation.AtStation or BinLocation.PickReturn;
    public bool IsOnRestockLoop => Location is BinLocation.RestockStation or BinLocation.RestockReturn;
}
