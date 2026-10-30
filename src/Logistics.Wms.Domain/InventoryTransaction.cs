namespace Logistics.Wms.Domain;

/// <summary>Stock ledger entry. Quantity is signed: + into stock, − out of stock.</summary>
public class InventoryTransaction
{
    public long Id { get; set; }
    public DateTime At { get; set; }
    public InventoryTransactionType Type { get; set; }
    public int SkuId { get; set; }
    public string? BinId { get; set; }
    public int Quantity { get; set; }
    /// <summary>Quantity in the bin after the booking (null for bookings outside a bin).</summary>
    public int? BinQuantityAfter { get; set; }
    public string? Reference { get; set; }
    public string? User { get; set; }
}
