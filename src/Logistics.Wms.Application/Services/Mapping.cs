using Logistics.Wms.Application.Contracts;
using Logistics.Wms.Domain;

namespace Logistics.Wms.Application.Services;

internal static class Mapping
{
    public static OrderDto ToDto(this Order o) => new(
        o.Id, o.Reference, o.Priority, o.Status, o.LaneId, o.CreatedAt, o.ActivatedAt, o.CompletedAt,
        o.TotalQuantity, o.Lines.Sum(l => l.Sorted),
        [.. o.Lines.OrderBy(l => l.Id).Select(l => new OrderLineDto(l.Id, l.Sku.Code, l.Quantity, l.Allocated, l.Picked, l.Sorted))]);

    public static BinDto ToDto(this Bin b) => new(
        b.Id, b.Sku.Code, b.Quantity, b.Sku.BinCapacity, b.Location, b.Slot?.Code, b.Station, b.Reserved, b.LastMovedAt);

    public static SlotDto ToDto(this StorageSlot s) => new(s.Id, s.Code, s.Column, s.Level, s.Angle, s.Height, s.BinId, s.Reserved, s.Blocked);

    public static ExceptionItemDto ToDto(this ExceptionItem e) => new(
        e.Id, e.PickTaskId, e.Sku.Code, e.OrderId, e.Reason, e.ReadBarcode, e.Disposition, e.CreatedAt, e.ResolvedAt, e.ResolvedBy);

    public static string Describe(this AsrsTaskKind kind) => kind switch
    {
        AsrsTaskKind.Retrieve => "retrieve",
        AsrsTaskKind.Store => "store",
        AsrsTaskKind.ReturnToRestock => "empty bin to restock",
        AsrsTaskKind.RackToRestock => "empty bin to restock",
        AsrsTaskKind.StoreRestocked => "store refilled",
        _ => kind.ToString(),
    };

    public static string Describe(this AsrsTaskKind kind, bool source, string? slot) => (kind, source) switch
    {
        (AsrsTaskKind.Retrieve or AsrsTaskKind.RackToRestock, true) => $"rack {slot}",
        (AsrsTaskKind.Retrieve, false) => "picking loop (east port)",
        (AsrsTaskKind.Store or AsrsTaskKind.ReturnToRestock, true) => "picking-loop return head",
        (AsrsTaskKind.ReturnToRestock or AsrsTaskKind.RackToRestock, false) => "restock loop (west port)",
        (AsrsTaskKind.StoreRestocked, true) => "restock-loop return head",
        (AsrsTaskKind.Store or AsrsTaskKind.StoreRestocked, false) => $"rack {slot}",
        _ => "?",
    };
}
