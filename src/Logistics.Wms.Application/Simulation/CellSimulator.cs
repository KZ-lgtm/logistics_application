using Logistics.Wms.Application.Abstractions;
using Logistics.Wms.Application.Contracts;
using Logistics.Wms.Application.Services;
using Logistics.Wms.Domain;
using Microsoft.EntityFrameworkCore;

namespace Logistics.Wms.Application.Simulation;

public sealed class SimulationOptions
{
    public bool Enabled { get; set; }
    /// <summary>Wall-clock time between two simulation rounds.</summary>
    public int StepMilliseconds { get; set; } = 500;
    public bool GenerateOrders { get; set; } = true;
    /// <summary>Queued orders the generator keeps in stock (the 3D simulation keeps 12).</summary>
    public int QueueTarget { get; set; } = 12;
    public int MaxLinesPerOrder { get; set; } = 4;
    public int MaxQuantityPerLine { get; set; } = 3;
    /// <summary>Pareto demand (Zipf, s = 0.9) or flat demand over all SKUs.</summary>
    public bool ParetoDemand { get; set; } = true;
    public double NoReadRate { get; set; } = 0.005;
    public int Seed { get; set; } = 7;
}

/// <summary>Mutable state of the simulator that outlives a single round (random stream, counters).</summary>
public sealed class SimulationState(SimulationOptions options)
{
    public SimulationOptions Options { get; } = options;
    public Random Random { get; } = new(options.Seed);
    public long Rounds { get; set; }
    public int OrdersGenerated { get; set; }
    public bool Running { get; set; } = options.Enabled;
}

/// <summary>
/// Stand-in for the cell controller (WCS / PLC) of the 3D simulation: every round each actor does at most one step through the
/// public WMS services — R1 executes one ASRS task, the picking loop routes bins, R2 / R3 pick one carton each, the arch scans,
/// S1 / S2 sort one carton each, the packing operator moves lane cases on, the restock operator refills one bin and the exception
/// tote is cleared. Used by the background service (live demo) and the end-to-end tests.
/// </summary>
public sealed class CellSimulator(
    IWmsDbContext db, SimulationState state, OrderService orders, AsrsService asrs, PickingService picking, SortationService sortation,
    PutWallService putWall, RestockService restock, ExceptionService exceptions)
{
    public async Task StepAsync(CancellationToken ct = default)
    {
        var o = state.Options;
        state.Rounds++;
        if (o.GenerateOrders) await GenerateOrdersAsync(ct);

        // R1
        if (await asrs.RequestNextTaskAsync(ct) is { } task) await asrs.CompleteTaskAsync(task.Id, ct);

        // picking loop: bins at the decision point, then arrivals at their stop
        foreach (var bin in await db.Bins.Where(b => b.Location == BinLocation.PickInbound).OrderBy(b => b.LastMovedAt).ThenBy(b => b.Id).Select(b => new { b.Id, b.Station }).ToListAsync(ct))
        {
            var station = bin.Station;
            if (station is null)
            {
                var route = await picking.RouteBinAsync(bin.Id, ct);
                if (route.Decision != RouteDecision.Station) continue;
                station = route.Station;
            }
            if (!await db.Bins.AnyAsync(b => b.Location == BinLocation.AtStation && b.Station == station, ct))
                await picking.ArriveAtStationAsync(bin.Id, station!.Value, ct);
        }

        // R2 / R3: one carton each
        foreach (var s in Enum.GetValues<PickStation>())
        {
            var st = await picking.GetStationAsync(s, ct);
            if (st.Picks.Count > 0) await picking.ConfirmPickAsync(st.Picks[0].PickTaskId, ct);
        }

        // bar-code arch
        var tote = await db.CellState.SingleAsync(ct);
        foreach (var p in await db.PickTasks.Where(p => p.Status == PickStatus.OnBelt).OrderBy(p => p.Id).Select(p => new { p.Id, p.SkuId }).ToListAsync(ct))
        {
            if (tote.ExceptionToteCount >= CellLayout.ExceptionToteCapacity) break;              // belt holds: tote full
            var code = (await db.Skus.SingleAsync(x => x.Id == p.SkuId, ct)).Code;
            await sortation.ScanAsync(new ScanRequest(p.Id, state.Random.NextDouble() < o.NoReadRate ? null : code), ct);
        }

        // S1 / S2: one carton each
        foreach (var sorter in Enum.GetValues<Sorter>())
        {
            var next = await db.PickTasks.Where(p => p.Status == PickStatus.Routed && p.TargetSorter == sorter).OrderBy(p => p.Id)
                .Select(p => new { p.Id, p.TargetLaneId }).FirstOrDefaultAsync(ct);
            if (next is not null) await sortation.ConfirmSortAsync(new SortConfirmRequest(next.Id, next.TargetLaneId!), ct);
        }

        // packing operator: move every finished / swapped case one stage on
        foreach (var lane in await db.Lanes.Where(l => l.State != LaneState.Ready && l.State != LaneState.Active).Select(l => new { l.Id, l.State }).ToListAsync(ct))
        {
            var ev = lane.State switch
            {
                LaneState.Outbound => LaneEvent.CaseAtOperator,
                LaneState.AtOperator => LaneEvent.CaseReplaced,
                _ => LaneEvent.EmptyCaseInPosition,
            };
            await putWall.AdvanceAsync(lane.Id, ev, ct);
        }

        // restock operator: one bin per round
        if ((await restock.ListAsync(ct)).FirstOrDefault() is { } refill) await restock.CompleteAsync(refill.BinId, new RestockRequest(User: "OP1"), ct);

        // put-wall operator: clears the exception tote, returns the cartons to stock
        foreach (var e in await exceptions.ListAsync(openOnly: true, take: 50, ct))
            await exceptions.ResolveAsync(e.Id, new ResolveExceptionRequest(ExceptionDisposition.ReturnedToStock, "OP2"), ct);
        if (tote.ExceptionToteCount > 0) await exceptions.EmptyToteAsync(ct);
    }

    private async Task GenerateOrdersAsync(CancellationToken ct)
    {
        var o = state.Options;
        var queued = await db.Orders.CountAsync(x => x.Status == OrderStatus.Queued, ct);
        if (queued >= o.QueueTarget) return;
        var skus = await db.Skus.OrderBy(s => s.Id).Select(s => s.Code).ToListAsync(ct);
        var weights = skus.Select((_, i) => o.ParetoDemand ? 1 / Math.Pow(i + 1, 0.9) : 1.0).ToArray();
        var sum = weights.Sum();

        for (var k = queued; k < o.QueueTarget; k++)
        {
            var r = state.Random;
            var lineCount = 1 + (int)Math.Floor(Math.Pow(r.NextDouble(), 1.6) * o.MaxLinesPerOrder);
            var lines = new List<CreateOrderLineRequest>();
            for (var tries = 0; lines.Count < lineCount && tries < 100; tries++)
            {
                var u = r.NextDouble() * sum;
                var i = 0;
                while (i < weights.Length - 1 && u > weights[i]) u -= weights[i++];
                if (lines.Any(l => l.SkuCode == skus[i])) continue;
                lines.Add(new CreateOrderLineRequest(skus[i], 1 + (int)Math.Floor(Math.Pow(r.NextDouble(), 2.2) * o.MaxQuantityPerLine)));
            }
            state.OrdersGenerated++;
            await orders.CreateAsync(new CreateOrderRequest($"SIM-{Guid.NewGuid():N}"[..16], lines), ct);
        }
    }
}
