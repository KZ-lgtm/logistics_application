using Logistics.Wms.Application.Abstractions;
using Logistics.Wms.Application.Contracts;
using Logistics.Wms.Domain;
using Microsoft.EntityFrameworkCore;

namespace Logistics.Wms.Application.Services;

/// <summary>Picking loop and the two picking stops P1 / P2 (R2 at P1, R3 at P2, 3D vision and vacuum grippers).</summary>
public sealed class PickingService(IWmsDbContext db, DemandService demand, RobotTracker robots, TimeProvider clock)
{
    /// <summary>
    /// Decision point before P1, taken when the bin can leave it: P2 unless a bin is already at / heading to P2, else P1 (a bin for
    /// P1 never waits behind one for P2); a stop whose picker is not available is skipped. No open demand for the SKU any more →
    /// straight back to the return head.
    /// </summary>
    public async Task<BinRouteDto> RouteBinAsync(string binId, CancellationToken ct = default)
    {
        var bin = await LoadBinAsync(binId, ct);
        if (bin.Location != BinLocation.PickInbound) throw WmsException.Conflict($"Bin {binId} is not on the picking loop inbound ({bin.Location}).");
        if (bin.Station is PickStation routed) return new BinRouteDto(bin.Id, RouteDecision.Station, routed, await demand.OpenQuantityAsync(bin.SkuId, ct));

        var open = await demand.OpenQuantityAsync(bin.SkuId, ct);
        if (open <= 0 || bin.IsEmpty)
        {
            MoveToReturn(bin, clock.GetUtcNow().UtcDateTime);
            await db.SaveChangesAsync(ct);
            return new BinRouteDto(bin.Id, RouteDecision.Return, null, open);
        }

        var busy = await db.Bins.Where(b => b.Id != bin.Id && b.Station != null && (b.Location == BinLocation.PickInbound || b.Location == BinLocation.AtStation))
            .Select(b => b.Station!.Value).ToListAsync(ct);
        PickStation? station = null;
        foreach (var candidate in new[] { PickStation.P2, PickStation.P1 })
        {
            if (busy.Contains(candidate) || !await robots.IsAvailableAsync(CellLayout.PickerOf(candidate), ct)) continue;
            station = candidate;
            break;
        }
        if (station is null) return new BinRouteDto(bin.Id, RouteDecision.Wait, null, open);

        bin.Station = station;
        await db.SaveChangesAsync(ct);
        return new BinRouteDto(bin.Id, RouteDecision.Station, station, open);
    }

    /// <summary>
    /// The bin stands at its stop: allocate its cartons to the wave. Without any allocation, or when the stop's picker is not available,
    /// the bin is released at once.
    /// </summary>
    public async Task<StationDto> ArriveAtStationAsync(string binId, PickStation station, CancellationToken ct = default)
    {
        var bin = await LoadBinAsync(binId, ct);
        if (bin.Location != BinLocation.PickInbound) throw WmsException.Conflict($"Bin {binId} is not on the picking loop inbound ({bin.Location}).");
        if (bin.Station is PickStation routed && routed != station) throw WmsException.Conflict($"Bin {binId} is routed to {routed}, not {station}.");
        var occupant = await db.Bins.FirstOrDefaultAsync(b => b.Location == BinLocation.AtStation && b.Station == station, ct);
        if (occupant is not null) throw WmsException.Conflict($"Stop {station} is occupied by bin {occupant.Id}.");

        bin.Station = station;
        bin.Location = BinLocation.AtStation;
        bin.LastMovedAt = clock.GetUtcNow().UtcDateTime;
        var picker = CellLayout.PickerOf(station);
        var picks = await robots.IsAvailableAsync(picker, ct) ? await demand.AllocateAsync(bin, ct) : [];
        if (picks.Count == 0) MoveToReturn(bin, bin.LastMovedAt.Value);
        await db.SaveChangesAsync(ct);
        if (picks.Count > 0)
        {
            await robots.BeginAsync(picker, $"pick:{picks[0].Id}", PickText(bin, picks[0]), ct);
            await db.SaveChangesAsync(ct);
        }
        return await GetStationAsync(station, ct, picks.Count == 0 ? null : bin);
    }

    public async Task<StationDto> GetStationAsync(PickStation station, CancellationToken ct = default, Bin? known = null)
    {
        var bin = known ?? await db.Bins.Include(b => b.Sku).FirstOrDefaultAsync(b => b.Location == BinLocation.AtStation && b.Station == station, ct);
        if (bin is null) return new StationDto(station, null, null, 0, []);
        var picks = await db.PickTasks.Where(p => p.BinId == bin.Id && p.Station == station && p.Status == PickStatus.Pending)
            .OrderBy(p => p.Sequence)
            .Select(p => new PickInstructionDto(p.Id, p.Sequence, p.OrderId, p.OrderLine.Order.Reference, bin.Sku.Code, p.Status))
            .ToListAsync(ct);
        return new StationDto(station, bin.Id, bin.Sku.Code, bin.Quantity, picks);
    }

    /// <summary>R2 / R3 placed the carton on the item belt. The id of the pick task tracks the carton from here on.</summary>
    public async Task<PickConfirmationDto> ConfirmPickAsync(long pickTaskId, CancellationToken ct = default)
    {
        var pick = await db.PickTasks.Include(p => p.OrderLine).SingleOrDefaultAsync(p => p.Id == pickTaskId, ct)
            ?? throw WmsException.NotFound("Pick task", pickTaskId);
        if (pick.Status != PickStatus.Pending) throw WmsException.Conflict($"Pick task {pickTaskId} is {pick.Status}.");
        var bin = await LoadBinAsync(pick.BinId, ct);
        if (bin.Location != BinLocation.AtStation || bin.Station != pick.Station) throw WmsException.Conflict($"Bin {bin.Id} is not at stop {pick.Station}.");
        if (bin.Quantity <= 0) throw WmsException.Conflict($"Bin {bin.Id} is empty; report the counted quantity.");

        var now = clock.GetUtcNow().UtcDateTime;
        bin.Quantity--;
        pick.Status = PickStatus.OnBelt;
        pick.PickedAt = now;
        pick.OrderLine.Picked++;
        db.InventoryTransactions.Add(new InventoryTransaction
        {
            At = now, Type = InventoryTransactionType.Pick, SkuId = bin.SkuId, BinId = bin.Id, Quantity = -1,
            BinQuantityAfter = bin.Quantity, Reference = $"order {pick.OrderId} / pick {pick.Id}",
        });

        var next = await db.PickTasks.Where(p => p.BinId == bin.Id && p.Status == PickStatus.Pending && p.Id != pick.Id)
            .OrderBy(p => p.Sequence).ToListAsync(ct);
        if (next.Count == 0) MoveToReturn(bin, now);
        var picker = CellLayout.PickerOf(pick.Station);
        await robots.EndAsync(picker, null, ct);
        if (next.Count > 0) await robots.BeginAsync(picker, $"pick:{next[0].Id}", PickText(bin, next[0]), ct);
        await db.SaveChangesAsync(ct);
        return new PickConfirmationDto(pick.Id, bin.Id, bin.Quantity, next.Count, next.Count == 0);
    }

    /// <summary>
    /// The 3D image shows a different number of cartons than booked (short or over stock). The bin quantity is corrected; pending
    /// picks the bin cannot serve any more are released back to demand, so another bin is fetched for them.
    /// </summary>
    public async Task<StationDto> ReportBinCountAsync(string binId, BinCountRequest request, CancellationToken ct = default)
    {
        if (request.CountedQuantity < 0) throw WmsException.Invalid("Counted quantity must not be negative.");
        var bin = await LoadBinAsync(binId, ct);
        if (bin.Location != BinLocation.AtStation || bin.Station is null) throw WmsException.Conflict($"Bin {binId} is not at a picking stop.");
        if (request.CountedQuantity > bin.Sku.BinCapacity) throw WmsException.Invalid($"Bin capacity is {bin.Sku.BinCapacity}.");

        var now = clock.GetUtcNow().UtcDateTime;
        var delta = request.CountedQuantity - bin.Quantity;
        if (delta != 0)
        {
            bin.Quantity = request.CountedQuantity;
            db.InventoryTransactions.Add(new InventoryTransaction
            {
                At = now, Type = InventoryTransactionType.Adjustment, SkuId = bin.SkuId, BinId = bin.Id, Quantity = delta,
                BinQuantityAfter = bin.Quantity, Reference = $"count at stop {bin.Station}", User = request.User,
            });
        }

        var pending = await db.PickTasks.Include(p => p.OrderLine)
            .Where(p => p.BinId == bin.Id && p.Status == PickStatus.Pending)
            .OrderBy(p => p.Sequence).ToListAsync(ct);
        foreach (var p in pending.Skip(bin.Quantity))
        {
            p.Status = PickStatus.Cancelled;
            p.OrderLine.Allocated--;
        }
        var station = bin.Station.Value;
        if (Math.Min(pending.Count, bin.Quantity) == 0)
        {
            MoveToReturn(bin, now);
            await robots.AbortAsync(CellLayout.PickerOf(station), null, ct);
        }
        await db.SaveChangesAsync(ct);
        return await GetStationAsync(station, ct);
    }

    internal static string PickText(Bin bin, PickTask pick) => $"pick {bin.Sku?.Code} from bin {bin.Id} for order {pick.OrderId}";

    internal static void MoveToReturn(Bin bin, DateTime now)
    {
        bin.LastMovedAt = now;
        bin.Location = BinLocation.PickReturn;
        bin.Station = null;
        bin.Allocated = false;
    }

    private async Task<Bin> LoadBinAsync(string binId, CancellationToken ct) =>
        await db.Bins.Include(b => b.Sku).SingleOrDefaultAsync(b => b.Id == binId, ct) ?? throw WmsException.NotFound("Bin", binId);
}
