using Logistics.Wms.Application.Abstractions;
using Logistics.Wms.Application.Contracts;
using Logistics.Wms.Domain;
using Microsoft.EntityFrameworkCore;

namespace Logistics.Wms.Application.Services;

/// <summary>
/// Item belt, bar-code arch and the sorters S1 / S2. A carton whose bar code cannot be read, does not match the picked SKU or
/// whose order is no longer on the put wall runs off the belt end into the exception tote; its order line goes back to demand.
/// </summary>
public sealed class SortationService(IWmsDbContext db, TimeProvider clock)
{
    public async Task<ScanResultDto> ScanAsync(ScanRequest request, CancellationToken ct = default)
    {
        var pick = await LoadAsync(request.PickTaskId, ct);
        if (pick.Status != PickStatus.OnBelt) throw WmsException.Conflict($"Pick task {pick.Id} is {pick.Status}, not on the belt.");
        pick.ScannedAt = clock.GetUtcNow().UtcDateTime;

        var sku = await db.Skus.SingleAsync(s => s.Id == pick.SkuId, ct);
        var order = await db.Orders.SingleAsync(o => o.Id == pick.OrderId, ct);
        ExceptionReason? reason =
            string.IsNullOrWhiteSpace(request.Barcode) ? ExceptionReason.NoRead
            : !string.Equals(request.Barcode.Trim(), sku.Code, StringComparison.OrdinalIgnoreCase) ? ExceptionReason.BarcodeMismatch
            : order.Status != OrderStatus.Active || order.LaneId is null ? ExceptionReason.OrderNotActive
            : null;

        if (reason is ExceptionReason r)
        {
            await ToExceptionAsync(pick, r, request.Barcode, ct);
            await db.SaveChangesAsync(ct);
            return new ScanResultDto(pick.Id, ScanTarget.Exception, null, pick.OrderId, r);
        }

        var lane = await db.Lanes.SingleAsync(l => l.Id == order.LaneId, ct);
        pick.Status = PickStatus.Routed;
        pick.TargetSorter = lane.Sorter;
        pick.TargetLaneId = lane.Id;
        await db.SaveChangesAsync(ct);
        return new ScanResultDto(pick.Id, lane.Sorter == Sorter.S1 ? ScanTarget.S1 : ScanTarget.S2, lane.Id, order.Id, null);
    }

    /// <summary>A routed carton was not taken by its sorter and ran off the belt end.</summary>
    public async Task<ScanResultDto> ReportMissedAsync(long pickTaskId, CancellationToken ct = default)
    {
        var pick = await LoadAsync(pickTaskId, ct);
        if (pick.Status is not (PickStatus.Routed or PickStatus.OnBelt)) throw WmsException.Conflict($"Pick task {pick.Id} is {pick.Status}.");
        await ToExceptionAsync(pick, ExceptionReason.MissedSorter, null, ct);
        await db.SaveChangesAsync(ct);
        return new ScanResultDto(pick.Id, ScanTarget.Exception, null, pick.OrderId, ExceptionReason.MissedSorter);
    }

    /// <summary>S1 / S2 put the carton into the order case; the last carton finishes the order and sends the case out.</summary>
    public async Task<SortConfirmationDto> ConfirmSortAsync(SortConfirmRequest request, CancellationToken ct = default)
    {
        var pick = await LoadAsync(request.PickTaskId, ct);
        if (pick.Status != PickStatus.Routed) throw WmsException.Conflict($"Pick task {pick.Id} is {pick.Status}, not routed.");
        if (!string.Equals(pick.TargetLaneId, request.LaneId, StringComparison.OrdinalIgnoreCase))
            throw WmsException.Conflict($"Pick task {pick.Id} belongs in lane {pick.TargetLaneId}, not {request.LaneId}.");
        var lane = await db.Lanes.SingleAsync(l => l.Id == pick.TargetLaneId, ct);
        if (lane.State != LaneState.Active || lane.OrderId != pick.OrderId) throw WmsException.Conflict($"Lane {lane.Id} is not collecting order {pick.OrderId}.");

        var now = clock.GetUtcNow().UtcDateTime;
        pick.Status = PickStatus.Sorted;
        pick.SortedAt = now;
        pick.OrderLine.Sorted++;

        var order = await db.Orders.Include(o => o.Lines).SingleAsync(o => o.Id == pick.OrderId, ct);
        var completed = order.IsFullySorted;
        if (completed)
        {
            order.Status = OrderStatus.Completed;
            order.CompletedAt = now;
            lane.OrderId = null;
            lane.State = LaneState.Outbound;
            lane.StateSince = now;
        }
        await db.SaveChangesAsync(ct);
        return new SortConfirmationDto(pick.Id, lane.Id, order.Id, completed);
    }

    /// <summary>Cartons on the belt and their routing (for the WCS after a restart).</summary>
    public async Task<IReadOnlyList<ScanResultDto>> ListOnBeltAsync(CancellationToken ct = default) =>
        await db.PickTasks.Where(p => p.Status == PickStatus.OnBelt || p.Status == PickStatus.Routed).OrderBy(p => p.PickedAt).ThenBy(p => p.Id)
            .Select(p => new ScanResultDto(p.Id,
                p.TargetSorter == null ? ScanTarget.Exception : p.TargetSorter == Sorter.S1 ? ScanTarget.S1 : ScanTarget.S2,
                p.TargetLaneId, p.OrderId, null))
            .ToListAsync(ct);

    private async Task ToExceptionAsync(PickTask pick, ExceptionReason reason, string? barcode, CancellationToken ct)
    {
        var now = clock.GetUtcNow().UtcDateTime;
        pick.Status = PickStatus.Exception;
        pick.TargetSorter = null;
        pick.TargetLaneId = null;
        pick.OrderLine.Allocated--;                              // the order stays active: the line is fetched again from a bin
        pick.OrderLine.Picked--;
        db.ExceptionItems.Add(new ExceptionItem
        {
            PickTaskId = pick.Id, SkuId = pick.SkuId, OrderId = pick.OrderId, Reason = reason, ReadBarcode = barcode,
            Disposition = ExceptionDisposition.Open, CreatedAt = now,
        });
        (await db.CellState.SingleAsync(ct)).ExceptionToteCount++;
    }

    private async Task<PickTask> LoadAsync(long id, CancellationToken ct) =>
        await db.PickTasks.Include(p => p.OrderLine).SingleOrDefaultAsync(p => p.Id == id, ct) ?? throw WmsException.NotFound("Pick task", id);
}
