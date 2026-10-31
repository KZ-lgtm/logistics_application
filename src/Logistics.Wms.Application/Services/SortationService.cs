using Logistics.Wms.Application.Abstractions;
using Logistics.Wms.Application.Contracts;
using Logistics.Wms.Domain;
using Microsoft.EntityFrameworkCore;

namespace Logistics.Wms.Application.Services;

/// <summary>
/// Item belt, bar-code arch and the sorters S1 / S2. A carton whose bar code cannot be read, does not match the picked SKU or
/// whose order is no longer on the put wall runs off the belt end into the exception tote; its order line goes back to demand.
/// So does a carton whose sorter is not available (its order gets no new picks until the sorter is back).
/// </summary>
public sealed class SortationService(IWmsDbContext db, RobotTracker robots, TimeProvider clock)
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

        var lane = reason is null ? await db.Lanes.SingleAsync(l => l.Id == order.LaneId, ct) : null;
        if (lane is not null && !await robots.IsAvailableAsync(CellLayout.SorterRobotOf(lane.Sorter), ct)) reason = ExceptionReason.SorterUnavailable;

        if (reason is ExceptionReason r)
        {
            await ToExceptionAsync(pick, r, request.Barcode, ct);
            await db.SaveChangesAsync(ct);
            return new ScanResultDto(pick.Id, ScanTarget.Exception, null, pick.OrderId, r);
        }

        pick.Status = PickStatus.Routed;
        pick.TargetSorter = lane!.Sorter;
        pick.TargetLaneId = lane.Id;
        var sorter = await robots.GetAsync(CellLayout.SorterRobotOf(lane.Sorter), ct);
        if (sorter.TaskRef is null) await robots.BeginAsync(sorter.Id, $"sort:{pick.Id}", SortText(sku.Code, pick), ct);
        await db.SaveChangesAsync(ct);
        return new ScanResultDto(pick.Id, lane.Sorter == Sorter.S1 ? ScanTarget.S1 : ScanTarget.S2, lane.Id, order.Id, null);
    }

    /// <summary>A routed carton was not taken by its sorter and ran off the belt end.</summary>
    public async Task<ScanResultDto> ReportMissedAsync(long pickTaskId, CancellationToken ct = default)
    {
        var pick = await LoadAsync(pickTaskId, ct);
        if (pick.Status is not (PickStatus.Routed or PickStatus.OnBelt)) throw WmsException.Conflict($"Pick task {pick.Id} is {pick.Status}.");
        var sorter = pick.TargetSorter;
        await ToExceptionAsync(pick, ExceptionReason.MissedSorter, null, ct);
        if (sorter is Sorter missedBy)
        {
            await robots.AbortAsync(CellLayout.SorterRobotOf(missedBy), $"sort:{pick.Id}", ct);
            await BeginNextAsync(missedBy, pick.Id, ct);
        }
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
        await robots.EndAsync(CellLayout.SorterRobotOf(lane.Sorter), null, ct);
        await BeginNextAsync(lane.Sorter, pick.Id, ct);
        await db.SaveChangesAsync(ct);
        return new SortConfirmationDto(pick.Id, lane.Id, order.Id, completed);
    }

    /// <summary>Cartons on the belt in belt order, with their routing once scanned (for the WCS after a restart, and the console).</summary>
    public async Task<IReadOnlyList<BeltItemDto>> ListOnBeltAsync(CancellationToken ct = default) =>
        await (from p in db.PickTasks
               join s in db.Skus on p.SkuId equals s.Id
               where p.Status == PickStatus.OnBelt || p.Status == PickStatus.Routed
               orderby p.PickedAt, p.Id
               select new BeltItemDto(p.Id, s.Code, p.OrderId, p.Status, p.TargetSorter, p.TargetLaneId, p.PickedAt))
            .ToListAsync(ct);

    internal static string SortText(string skuCode, PickTask pick) => $"put {skuCode} (carton #{pick.Id}) into lane {pick.TargetLaneId}";

    /// <summary>The sorter's next task: the oldest carton routed to it.</summary>
    private async Task BeginNextAsync(Sorter sorter, long done, CancellationToken ct)
    {
        var next = await (from p in db.PickTasks
                          join s in db.Skus on p.SkuId equals s.Id
                          where p.Status == PickStatus.Routed && p.TargetSorter == sorter && p.Id != done
                          orderby p.ScannedAt, p.Id
                          select new { Pick = p, s.Code }).FirstOrDefaultAsync(ct);
        if (next is not null) await robots.BeginAsync(CellLayout.SorterRobotOf(sorter), $"sort:{next.Pick.Id}", SortText(next.Code, next.Pick), ct);
    }

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
