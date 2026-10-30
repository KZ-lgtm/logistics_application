namespace Logistics.Wms.Domain;

public class Order
{
    public int Id { get; set; }
    /// <summary>Reference of the order in the host system (ERP / web shop).</summary>
    public required string Reference { get; set; }
    /// <summary>Higher values are released to the put wall first; FIFO within a priority.</summary>
    public int Priority { get; set; }
    public OrderStatus Status { get; set; }
    public string? LaneId { get; set; }
    public DateTime CreatedAt { get; set; }
    public DateTime? ActivatedAt { get; set; }
    public DateTime? CompletedAt { get; set; }
    public List<OrderLine> Lines { get; set; } = [];

    public int TotalQuantity => Lines.Sum(l => l.Quantity);
    public bool IsFullySorted => Lines.All(l => l.Sorted >= l.Quantity);
}

public class OrderLine
{
    public int Id { get; set; }
    public int OrderId { get; set; }
    public Order Order { get; set; } = null!;
    public int SkuId { get; set; }
    public Sku Sku { get; set; } = null!;
    public int Quantity { get; set; }
    /// <summary>Pieces allocated to a bin presentation (pending or picked).</summary>
    public int Allocated { get; set; }
    public int Picked { get; set; }
    public int Sorted { get; set; }

    /// <summary>Pieces that still need a bin.</summary>
    public int OpenQuantity => Quantity - Allocated;
}
