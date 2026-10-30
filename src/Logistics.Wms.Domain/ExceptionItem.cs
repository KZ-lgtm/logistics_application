namespace Logistics.Wms.Domain;

/// <summary>A carton in the exception tote at the end of the item belt.</summary>
public class ExceptionItem
{
    public int Id { get; set; }
    public long PickTaskId { get; set; }
    public int SkuId { get; set; }
    public Sku Sku { get; set; } = null!;
    public int OrderId { get; set; }
    public ExceptionReason Reason { get; set; }
    /// <summary>Bar code read at the arch (null on a no-read).</summary>
    public string? ReadBarcode { get; set; }
    public ExceptionDisposition Disposition { get; set; }
    public DateTime CreatedAt { get; set; }
    public DateTime? ResolvedAt { get; set; }
    public string? ResolvedBy { get; set; }
}
