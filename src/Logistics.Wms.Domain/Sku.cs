namespace Logistics.Wms.Domain;

public class Sku
{
    public int Id { get; set; }
    /// <summary>Bar-code content printed on every carton of this SKU (e.g. "SKU-1001").</summary>
    public required string Code { get; set; }
    public required string Name { get; set; }
    public SkuSize Size { get; set; }
    public double WeightKg { get; set; }
    /// <summary>Quantity below which the SKU is reported as low stock.</summary>
    public int ReorderPoint { get; set; }
    /// <summary>Cartons returned from the exception tote, waiting at the restock operator to be put into a bin of this SKU.</summary>
    public int RestockBuffer { get; set; }

    public int BinCapacity => CellLayout.BinCapacity(Size);
}
