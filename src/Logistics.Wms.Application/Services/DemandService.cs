using Logistics.Wms.Application.Abstractions;
using Logistics.Wms.Domain;
using Microsoft.EntityFrameworkCore;

namespace Logistics.Wms.Application.Services;

/// <summary>
/// Wave demand: the WMS adds up the open lines of all active orders (one per put-wall lane) per SKU and asks R1 for bins until
/// the bins already on their way to a picking stop cover it. Picks are allocated when a bin arrives at the stop, oldest order first.
/// Orders on the lanes of a sorter that is down get no new picks until it is back (their cartons could not be put into the case).
/// </summary>
public sealed class DemandService(IWmsDbContext db, RobotTracker robots, TimeProvider clock)
{
    /// <summary>Open lines of active orders whose sorter can take their cartons.</summary>
    private async Task<IQueryable<OrderLine>> OpenLinesAsync(CancellationToken ct)
    {
        var (_, blockedLanes) = await robots.UnavailableSortersAsync(ct);
        var q = db.OrderLines.Where(l => l.Order.Status == OrderStatus.Active && l.Quantity > l.Allocated);
        return blockedLanes.Count == 0 ? q : q.Where(l => !blockedLanes.Contains(l.Order.LaneId!));
    }

    /// <summary>Open (unallocated) quantity per SKU over all active orders.</summary>
    public async Task<Dictionary<int, int>> OpenQuantityBySkuAsync(CancellationToken ct = default) =>
        await (await OpenLinesAsync(ct))
            .GroupBy(l => l.SkuId)
            .Select(g => new { g.Key, Open = g.Sum(l => l.Quantity - l.Allocated) })
            .ToDictionaryAsync(x => x.Key, x => x.Open, ct);

    public async Task<int> OpenQuantityAsync(int skuId, CancellationToken ct = default) =>
        await (await OpenLinesAsync(ct))
            .Where(l => l.SkuId == skuId)
            .SumAsync(l => l.Quantity - l.Allocated, ct);

    /// <summary>Quantity per SKU in bins heading for a stop and not yet allocated (incl. a retrieval R1 is executing).</summary>
    public async Task<Dictionary<int, int>> CoveredQuantityBySkuAsync(CancellationToken ct = default)
    {
        var retrieving = db.AsrsTasks.Where(t => t.Status == AsrsTaskStatus.Assigned && t.Kind == AsrsTaskKind.Retrieve).Select(t => t.BinId);
        return await db.Bins
            .Where(b => !b.Allocated && (b.Location == BinLocation.PickInbound || retrieving.Contains(b.Id)))
            .GroupBy(b => b.SkuId)
            .Select(g => new { g.Key, Qty = g.Sum(b => b.Quantity) })
            .ToDictionaryAsync(x => x.Key, x => x.Qty, ct);
    }

    /// <summary>
    /// The SKU R1 should fetch next: first line (oldest active order first) whose SKU demand is not covered by bins in transit and
    /// for which a stocked bin is available in the rack; null when nothing is needed.
    /// </summary>
    public async Task<int?> NextNeededSkuAsync(CancellationToken ct = default)
    {
        var open = await OpenQuantityBySkuAsync(ct);
        if (open.Count == 0) return null;
        var covered = await CoveredQuantityBySkuAsync(ct);
        var available = (await db.Bins
                .Where(b => b.Location == BinLocation.Rack && b.Quantity > 0 && !b.Reserved)
                .Select(b => b.SkuId).Distinct().ToListAsync(ct))
            .ToHashSet();

        var lines = await (await OpenLinesAsync(ct))
            .OrderBy(l => l.Order.ActivatedAt).ThenBy(l => l.OrderId).ThenBy(l => l.Id)
            .Select(l => l.SkuId)
            .ToListAsync(ct);

        foreach (var sku in lines)
        {
            if (open[sku] > covered.GetValueOrDefault(sku) && available.Contains(sku)) return sku;
        }
        return null;
    }

    /// <summary>
    /// Allocates the cartons of a bin that has arrived at a picking stop to the open lines of the active orders (oldest first),
    /// creating one pick task per carton. Saves nothing; the caller saves.
    /// </summary>
    public async Task<List<PickTask>> AllocateAsync(Bin bin, CancellationToken ct = default)
    {
        if (bin.Station is null) throw WmsException.Conflict($"Bin {bin.Id} is not at a picking stop.");
        var lines = await (await OpenLinesAsync(ct))
            .Include(l => l.Order)
            .Where(l => l.SkuId == bin.SkuId)
            .OrderBy(l => l.Order.ActivatedAt).ThenBy(l => l.OrderId).ThenBy(l => l.Id)
            .ToListAsync(ct);

        var now = clock.GetUtcNow().UtcDateTime;
        var picks = new List<PickTask>();
        foreach (var line in lines)
        {
            while (line.OpenQuantity > 0 && picks.Count < bin.Quantity)
            {
                line.Allocated++;
                picks.Add(new PickTask
                {
                    BinId = bin.Id, Station = bin.Station.Value, OrderId = line.OrderId, OrderLine = line, SkuId = bin.SkuId,
                    Sequence = picks.Count + 1, Status = PickStatus.Pending, CreatedAt = now,
                });
            }
            if (picks.Count >= bin.Quantity) break;
        }
        db.PickTasks.AddRange(picks);
        bin.Allocated = true;
        return picks;
    }
}
