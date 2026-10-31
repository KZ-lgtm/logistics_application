using Logistics.Wms.Application.Abstractions;
using Logistics.Wms.Application.Contracts;
using Logistics.Wms.Domain;
using Microsoft.EntityFrameworkCore;

namespace Logistics.Wms.Application.Services;

public sealed class DashboardService(
    IWmsDbContext db, PutWallService putWall, PickingService picking, AsrsService asrs, InventoryService inventory, RobotService robots, TimeProvider clock)
{
    /// <summary>Order counts and last-hour figures only (cheap enough to poll every second).</summary>
    public async Task<OrderFigures> GetOrderFiguresAsync(CancellationToken ct = default)
    {
        var hourAgo = clock.GetUtcNow().UtcDateTime.AddHours(-1);
        var byStatus = await db.Orders.Where(o => o.Status != OrderStatus.Cancelled).GroupBy(o => o.Status)
            .Select(g => new { g.Key, N = g.Count() }).ToDictionaryAsync(x => x.Key, x => x.N, ct);
        var lead = await db.Orders.Where(o => o.Status == OrderStatus.Completed && o.CompletedAt >= hourAgo)
            .Select(o => new { o.ActivatedAt, o.CompletedAt }).ToListAsync(ct);
        var archived = await db.CellState.Select(c => c.ArchivedOrdersCompleted).SingleAsync(ct);
        return new OrderFigures(byStatus.GetValueOrDefault(OrderStatus.Queued), byStatus.GetValueOrDefault(OrderStatus.Active),
            byStatus.GetValueOrDefault(OrderStatus.Completed) + archived,
            lead.Count == 0 ? null : lead.Average(o => (o.CompletedAt!.Value - o.ActivatedAt!.Value).TotalSeconds), lead.Count);
    }

    public async Task<DashboardDto> GetAsync(CancellationToken ct = default)
    {
        var now = clock.GetUtcNow().UtcDateTime;
        var hourAgo = now.AddHours(-1);
        var state = await db.CellState.SingleAsync(ct);

        var byStatus = await db.Orders.GroupBy(o => o.Status).Select(g => new { g.Key, N = g.Count() }).ToDictionaryAsync(x => x.Key, x => x.N, ct);
        var lead = await db.Orders.Where(o => o.Status == OrderStatus.Completed && o.CompletedAt >= hourAgo)
            .Select(o => new { o.ActivatedAt, o.CompletedAt }).ToListAsync(ct);
        double? avgLead = lead.Count == 0 ? null : lead.Average(o => (o.CompletedAt!.Value - o.ActivatedAt!.Value).TotalSeconds);

        var picks = await db.PickTasks.GroupBy(p => p.Status).Select(g => new { g.Key, N = g.Count() }).ToDictionaryAsync(x => x.Key, x => x.N, ct);
        var sortedLastHour = await db.PickTasks.CountAsync(p => p.Status == PickStatus.Sorted && p.SortedAt >= hourAgo, ct);
        var scans = await db.PickTasks.CountAsync(p => p.ScannedAt != null, ct) + state.ArchivedScans;
        var noReads = await db.ExceptionItems.CountAsync(e => e.Reason == ExceptionReason.NoRead, ct) + state.ArchivedNoReads;
        var pickedNow = picks.GetValueOrDefault(PickStatus.OnBelt) + picks.GetValueOrDefault(PickStatus.Routed)
            + picks.GetValueOrDefault(PickStatus.Sorted) + picks.GetValueOrDefault(PickStatus.Exception);
        var presentations = await db.PickTasks.Where(p => p.Status != PickStatus.Cancelled && p.Status != PickStatus.Pending)
            .Select(p => new { p.BinId, p.CreatedAt }).Distinct().CountAsync(ct);

        var bins = await db.Bins.Select(b => new { b.Location, b.Quantity }).ToListAsync(ct);
        var slots = await db.Slots.CountAsync(s => !s.Blocked, ct);
        var freeSlots = await db.Slots.CountAsync(s => s.BinId == null && !s.Reserved && !s.Blocked, ct);
        var buffer = await db.Skus.SumAsync(s => s.RestockBuffer, ct);

        return new DashboardDto(
            now,
            new OrderKpis(byStatus.GetValueOrDefault(OrderStatus.Queued), byStatus.GetValueOrDefault(OrderStatus.Active),
                byStatus.GetValueOrDefault(OrderStatus.Completed) + state.ArchivedOrdersCompleted,
                byStatus.GetValueOrDefault(OrderStatus.Cancelled) + state.ArchivedOrdersCancelled, avgLead),
            new ThroughputKpis(
                lead.Count, sortedLastHour, pickedNow + state.ArchivedItemsPicked, picks.GetValueOrDefault(PickStatus.Sorted) + state.ArchivedItemsSorted,
                picks.GetValueOrDefault(PickStatus.OnBelt) + picks.GetValueOrDefault(PickStatus.Routed),
                scans, noReads, scans == 0 ? 0 : (double)noReads / scans, presentations == 0 ? 0 : (double)pickedNow / presentations),
            new StorageKpis(slots, freeSlots,
                bins.Count(b => b.Location == BinLocation.Rack), bins.Count(b => b.Location == BinLocation.Rack && b.Quantity <= 0),
                bins.Count(b => b.Location is BinLocation.PickInbound or BinLocation.AtStation or BinLocation.PickReturn),
                bins.Count(b => b.Location is BinLocation.RestockStation or BinLocation.RestockReturn),
                bins.Sum(b => b.Quantity) + buffer),
            await putWall.ListAsync(ct),
            [await picking.GetStationAsync(PickStation.P1, ct), await picking.GetStationAsync(PickStation.P2, ct)],
            await asrs.GetCurrentTaskAsync(ct),
            new ExceptionKpis(await db.ExceptionItems.CountAsync(e => e.Disposition == ExceptionDisposition.Open, ct),
                state.ExceptionToteCount, CellLayout.ExceptionToteCapacity, state.ExceptionToteCount >= CellLayout.ExceptionToteCapacity),
            [.. (await inventory.GetStockAsync(ct)).Where(s => s.LowStock)],
            await robots.ListAsync(ct));
    }
}
