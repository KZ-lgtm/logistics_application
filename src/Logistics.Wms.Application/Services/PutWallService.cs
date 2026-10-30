using Logistics.Wms.Application.Abstractions;
using Logistics.Wms.Application.Contracts;
using Logistics.Wms.Domain;
using Microsoft.EntityFrameworkCore;

namespace Logistics.Wms.Application.Services;

/// <summary>
/// The 8-lane put wall and rolling waves: every lane with an empty case in position takes the next queued order (priority, then
/// FIFO); a finished case runs out to the packing operator, who swaps it for an empty one that runs back in.
/// </summary>
public sealed class PutWallService(IWmsDbContext db, TimeProvider clock)
{
    public async Task<IReadOnlyList<LaneDto>> ListAsync(CancellationToken ct = default)
    {
        var lanes = await db.Lanes.OrderBy(l => l.Sorter).ThenBy(l => l.Index).ToListAsync(ct);
        var orderIds = lanes.Where(l => l.OrderId != null).Select(l => l.OrderId!.Value).ToList();
        var orders = await db.Orders.Include(o => o.Lines).Where(o => orderIds.Contains(o.Id)).ToDictionaryAsync(o => o.Id, ct);
        return [.. lanes.Select(l =>
        {
            var o = l.OrderId is int id ? orders.GetValueOrDefault(id) : null;
            return new LaneDto(l.Id, l.Sorter, l.Index, l.State, l.Disabled, o?.Id, o?.Reference, o?.TotalQuantity ?? 0,
                o?.Lines.Sum(x => x.Sorted) ?? 0, l.CasesCompleted, l.StateSince);
        })];
    }

    /// <summary>Assigns queued orders to idle lanes. Saves nothing; the caller saves. Returns the activated orders.</summary>
    public async Task<List<Order>> ReleaseOrdersAsync(CancellationToken ct = default)
    {
        var lanes = await db.Lanes.Where(l => l.State == LaneState.Ready && !l.Disabled && l.OrderId == null)
            .OrderBy(l => l.Sorter).ThenBy(l => l.Index).ToListAsync(ct);
        if (lanes.Count == 0) return [];
        var queued = await db.Orders.Where(o => o.Status == OrderStatus.Queued)
            .OrderByDescending(o => o.Priority).ThenBy(o => o.CreatedAt).ThenBy(o => o.Id)
            .Take(lanes.Count).ToListAsync(ct);

        var now = clock.GetUtcNow().UtcDateTime;
        foreach (var (lane, order) in lanes.Zip(queued))
        {
            order.Status = OrderStatus.Active;
            order.LaneId = lane.Id;
            order.ActivatedAt = now;
            lane.OrderId = order.Id;
            lane.State = LaneState.Active;
            lane.StateSince = now;
        }
        return queued;
    }

    public async Task<LaneDto> AdvanceAsync(string laneId, LaneEvent laneEvent, CancellationToken ct = default)
    {
        var lane = await FindAsync(laneId, ct);
        var (expected, next) = laneEvent switch
        {
            LaneEvent.CaseAtOperator => (LaneState.Outbound, LaneState.AtOperator),
            LaneEvent.CaseReplaced => (LaneState.AtOperator, LaneState.Inbound),
            LaneEvent.EmptyCaseInPosition => (LaneState.Inbound, LaneState.Ready),
            _ => throw WmsException.Invalid($"Unknown lane event {laneEvent}."),
        };
        if (lane.State != expected) throw WmsException.Conflict($"Lane {lane.Id} is {lane.State}; {laneEvent} needs {expected}.");

        lane.State = next;
        lane.StateSince = clock.GetUtcNow().UtcDateTime;
        if (laneEvent == LaneEvent.CaseReplaced) lane.CasesCompleted++;
        await db.SaveChangesAsync(ct);
        if (next == LaneState.Ready)
        {
            await ReleaseOrdersAsync(ct);
            await db.SaveChangesAsync(ct);
        }
        return (await ListAsync(ct)).Single(l => l.Id == lane.Id);
    }

    /// <summary>A disabled lane finishes its current order but takes no new one.</summary>
    public async Task<LaneDto> SetEnabledAsync(string laneId, bool enabled, CancellationToken ct = default)
    {
        var lane = await FindAsync(laneId, ct);
        lane.Disabled = !enabled;
        await db.SaveChangesAsync(ct);
        if (enabled)
        {
            await ReleaseOrdersAsync(ct);
            await db.SaveChangesAsync(ct);
        }
        return (await ListAsync(ct)).Single(l => l.Id == lane.Id);
    }

    private async Task<PutWallLane> FindAsync(string laneId, CancellationToken ct) =>
        await db.Lanes.SingleOrDefaultAsync(l => l.Id == laneId, ct) ?? throw WmsException.NotFound("Lane", laneId);
}
