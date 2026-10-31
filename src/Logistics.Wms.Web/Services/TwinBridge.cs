using Logistics.Wms.Application.Contracts;
using Logistics.Wms.Application.Services;
using Logistics.Wms.Application.Simulation;
using Logistics.Wms.Domain;
using Logistics.Wms.Persistence;
using Microsoft.EntityFrameworkCore;
using Microsoft.JSInterop;

namespace Logistics.Wms.Web.Services;

/// <summary>
/// The live 3D cell (wwwroot/twin) is the WMS's cell controller: it asks the WMS for every decision (R1 tasks, routing, picks,
/// sorting targets) and books every event (task done, pick, scan, sort, lane steps, restock, exception tote). Each call carries the
/// simulation time, so the WMS clock follows the 3D cell, and runs as one WMS command. Replies are { ok, error, data }.
/// </summary>
public sealed class TwinBridge(WmsClient wms, SimClock clock, SimulationState simulation, TwinService twin, Localizer localizer)
{
    public sealed record Reply(bool Ok, string? Error, object? Data);

    public sealed record TwinSku(string Code, string Size);
    public sealed record TwinBin(string Id, int Sku, int Qty, string Loc, int? Col, int? Level, string? Station, DateTime? Moved);
    public sealed record TwinPick(long Id, int OrderId);
    public sealed record TwinLane(string Id, string State, int? OrderId, bool Disabled, string? Reference = null, int Items = 0, int Sorted = 0);
    public sealed record TwinQueued(int Id, string Reference, int Priority, int Items, string Lines);
    public sealed record TwinBeltItem(long Pick, int Sku, int OrderId, string Status, string? Sorter, string? Lane);
    public sealed record TwinRobot(string Id, bool Available, string Mode, string State, string? FaultCode, string? FaultText, string? Task, double Utilization, int Cycles);
    public sealed record TwinOrders(int Queued, int Active, int Completed, double? LeadSeconds, int LastHour);
    public sealed record TwinOptions(double NoReadRate, int MaxLines, int MaxQuantity, bool Pareto, bool GenerateOrders);
    public sealed record TwinInit(
        IReadOnlyList<TwinSku> Skus, IReadOnlyList<TwinBin> Bins, Dictionary<string, IReadOnlyList<TwinPick>> Stations, IReadOnlyList<TwinLane> Lanes,
        IReadOnlyList<TwinBeltItem> Belt, int Tote, IReadOnlyList<TwinRobot> Robots, TwinOrders Orders, TwinOptions Options, IReadOnlyList<TwinQueued> Queue, int? Focus, string Language);
    public sealed record TwinPoll(IReadOnlyList<TwinLane> Lanes, IReadOnlyList<TwinRobot> Robots, TwinOrders Orders, int Tote, TwinOptions Options, IReadOnlyList<TwinQueued> Queue);
    public sealed record TwinTask(int Id, string Kind, string BinId, string? FromSlot, string? ToSlot);
    public sealed record TwinRoute(string Decision, string? Station);
    public sealed record TwinScan(string Target, string? Lane, string? Reason);
    public sealed record TwinRestock(int Qty, int Sku);

    /// <summary>Complete state for building the 3D cell. A task R1 had in hand is cancelled: the cell starts it afresh.</summary>
    [JSInvokable]
    public Task<Reply> Init(double t) => Run(t, async sp =>
    {
        var db = sp.Get<WmsDbContext>();
        foreach (var task in await db.AsrsTasks.Where(x => x.Status == AsrsTaskStatus.Assigned).Select(x => x.Id).ToListAsync())
            await sp.Get<AsrsService>().CancelTaskAsync(task);

        var skus = await db.Skus.OrderBy(s => s.Id).ToListAsync();
        var index = skus.Select((s, i) => (s.Id, i)).ToDictionary(x => x.Id, x => x.i);
        var bins = await db.Bins.Include(b => b.Slot).OrderBy(b => b.Id).ToListAsync();
        var stations = new Dictionary<string, IReadOnlyList<TwinPick>>();
        foreach (var st in Enum.GetValues<PickStation>())
            stations[st.ToString()] = [.. (await sp.Get<PickingService>().GetStationAsync(st)).Picks.Select(p => new TwinPick(p.PickTaskId, p.OrderId))];
        var belt = await db.PickTasks.Where(p => p.Status == PickStatus.OnBelt || p.Status == PickStatus.Routed).OrderBy(p => p.PickedAt).ThenBy(p => p.Id).ToListAsync();

        return new TwinInit(
            [.. skus.Select(s => new TwinSku(s.Code, s.Size.ToString()))],
            [.. bins.Select(b => new TwinBin(b.Id, index[b.SkuId], b.Quantity, b.Location.ToString(), b.Slot?.Column, b.Slot?.Level, b.Station?.ToString(), b.LastMovedAt))],
            stations,
            await LanesAsync(sp),
            [.. belt.Select(p => new TwinBeltItem(p.Id, index[p.SkuId], p.OrderId, p.Status.ToString(), p.TargetSorter?.ToString(), p.TargetLaneId))],
            (await db.CellState.SingleAsync()).ExceptionToteCount,
            await RobotsAsync(sp),
            await OrdersAsync(sp),
            Options(),
            await QueueAsync(sp),
            twin.FocusOrderId,
            localizer.IsChinese ? "zh" : "en");
    });

    /// <summary>Once a second: put-wall lanes, robots, order figures; tops up the order queue when the generator is on.</summary>
    [JSInvokable]
    public Task<Reply> Poll(double t) => Run(t, async sp =>
    {
        if (simulation.Options.GenerateOrders) await sp.Get<CellSimulator>().GenerateOrdersAsync();
        return new TwinPoll(await LanesAsync(sp), await RobotsAsync(sp), await OrdersAsync(sp), (await sp.Get<WmsDbContext>().CellState.SingleAsync()).ExceptionToteCount, Options(),
            await QueueAsync(sp));
    });

    [JSInvokable]
    public Task<Reply> R1Next(double t, string? pickHeadBin, string? restockHeadBin, bool pickOutFree, bool restockOutFree) => Run(t, async sp =>
    {
        var task = await sp.Get<AsrsService>().RequestNextTaskAsync(new AsrsHeads(pickHeadBin, restockHeadBin, pickOutFree, restockOutFree));
        return task is null ? null : new TwinTask(task.Id, task.Kind.ToString(), task.BinId, task.FromSlot, task.ToSlot);
    });

    [JSInvokable]
    public Task<Reply> R1Complete(double t, int id) => Run(t, async sp => { await sp.Get<AsrsService>().CompleteTaskAsync(id); return true; });

    [JSInvokable]
    public Task<Reply> R1Cancel(double t, int id) => Run(t, async sp => { await sp.Get<AsrsService>().CancelTaskAsync(id); return true; });

    [JSInvokable]
    public Task<Reply> Route(double t, string binId) => Run(t, async sp =>
    {
        var r = await sp.Get<PickingService>().RouteBinAsync(binId);
        return new TwinRoute(r.Decision.ToString(), r.Station?.ToString());
    });

    [JSInvokable]
    public Task<Reply> Arrive(double t, string binId, string station) => Run(t, async sp =>
    {
        var st = await sp.Get<PickingService>().ArriveAtStationAsync(binId, Enum.Parse<PickStation>(station));
        return st.BinId == binId ? st.Picks.Select(p => new TwinPick(p.PickTaskId, p.OrderId)).ToList() : [];
    });

    /// <summary>A robot starts a task (cycle timing from the real motion).</summary>
    [JSInvokable]
    public Task<Reply> RobotBegin(double t, string robotId, string taskRef, string text) => Run(t, async sp =>
    {
        await sp.Get<RobotTracker>().BeginAsync(robotId.ToUpperInvariant(), taskRef, text);
        await sp.Get<WmsDbContext>().SaveChangesAsync();
        return true;
    });

    [JSInvokable]
    public Task<Reply> PickDone(double t, long pickId) => Run(t, async sp => { await sp.Get<PickingService>().ConfirmPickAsync(pickId); return true; });

    [JSInvokable]
    public Task<Reply> Scan(double t, long pickId, string? barcode) => Run(t, async sp =>
    {
        var r = await sp.Get<SortationService>().ScanAsync(new ScanRequest(pickId, barcode));
        return new TwinScan(r.Target.ToString(), r.LaneId, r.ExceptionReason?.ToString());
    });

    [JSInvokable]
    public Task<Reply> Sort(double t, long pickId, string laneId) => Run(t, async sp =>
        (await sp.Get<SortationService>().ConfirmSortAsync(new SortConfirmRequest(pickId, laneId))).OrderCompleted);

    [JSInvokable]
    public Task<Reply> Missed(double t, long pickId) => Run(t, async sp => { await sp.Get<SortationService>().ReportMissedAsync(pickId); return true; });

    [JSInvokable]
    public Task<Reply> Lane(double t, string laneId, string laneEvent) => Run(t, async sp =>
    {
        var l = await sp.Get<PutWallService>().AdvanceAsync(laneId, Enum.Parse<LaneEvent>(laneEvent));
        return new TwinLane(l.Id, l.State.ToString(), l.OrderId, l.Disabled, l.OrderReference, l.OrderItems, l.SortedItems);
    });

    [JSInvokable]
    public Task<Reply> Restock(double t, string binId) => Run(t, async sp =>
    {
        var bin = await sp.Get<RestockService>().CompleteAsync(binId, new RestockRequest(User: "OP1"));
        var skus = await sp.Get<WmsDbContext>().Skus.OrderBy(s => s.Id).Select(s => s.Code).ToListAsync();
        return new TwinRestock(bin.Quantity, skus.IndexOf(bin.SkuCode));
    });

    /// <summary>The put-wall operator emptied the exception tote: open exceptions go back to stock.</summary>
    [JSInvokable]
    public Task<Reply> ToteEmptied(double t) => Run(t, async sp =>
    {
        var exceptions = sp.Get<ExceptionService>();
        foreach (var e in await exceptions.ListAsync(openOnly: true, take: 500))
            await exceptions.ResolveAsync(e.Id, new ResolveExceptionRequest(ExceptionDisposition.ReturnedToStock, "OP2"));
        return await exceptions.EmptyToteAsync();
    });

    [JSInvokable]
    public Task<Reply> SetOption(string name, double value)
    {
        var o = simulation.Options;
        switch (name)
        {
            case "no-read": o.NoReadRate = Math.Clamp(value, 0, 20) / 100; break;
            case "lines-max": o.MaxLinesPerOrder = (int)Math.Clamp(Math.Round(value), 1, 8); break;
            case "qty-max": o.MaxQuantityPerLine = (int)Math.Clamp(Math.Round(value), 1, 10); break;
            case "pareto": o.ParetoDemand = value > 0; break;
            case "generate": o.GenerateOrders = value > 0; break;
            default: return Task.FromResult(new Reply(false, $"Unknown option {name}.", null));
        }
        return Task.FromResult(new Reply(true, null, Options()));
    }

    private TwinOptions Options()
    {
        var o = simulation.Options;
        return new TwinOptions(o.NoReadRate, o.MaxLinesPerOrder, o.MaxQuantityPerLine, o.ParetoDemand, o.GenerateOrders);
    }

    private static async Task<IReadOnlyList<TwinLane>> LanesAsync(IServiceProvider sp) =>
        [.. (await sp.Get<PutWallService>().ListAsync()).Select(l => new TwinLane(l.Id, l.State.ToString(), l.OrderId, l.Disabled, l.OrderReference, l.OrderItems, l.SortedItems))];

    /// <summary>The next queued orders in release order (priority, then FIFO) and their lines.</summary>
    private static async Task<IReadOnlyList<TwinQueued>> QueueAsync(IServiceProvider sp)
    {
        var orders = await sp.Get<WmsDbContext>().Orders.Include(o => o.Lines).ThenInclude(l => l.Sku)
            .Where(o => o.Status == OrderStatus.Queued)
            .OrderByDescending(o => o.Priority).ThenBy(o => o.CreatedAt).ThenBy(o => o.Id).Take(6).ToListAsync();
        return [.. orders.Select(o => new TwinQueued(o.Id, o.Reference, o.Priority, o.TotalQuantity, string.Join(", ", o.Lines.Select(l => $"{l.Sku.Code}×{l.Quantity}"))))];
    }

    private static async Task<IReadOnlyList<TwinRobot>> RobotsAsync(IServiceProvider sp) =>
        [.. (await sp.Get<RobotService>().ListAsync()).Select(r =>
            new TwinRobot(r.Id, r.Available, r.Mode.ToString(), r.State.ToString(), r.FaultCode, r.FaultText, r.TaskText, r.Utilization, r.Cycles))];

    private static async Task<TwinOrders> OrdersAsync(IServiceProvider sp)
    {
        var o = await sp.Get<DashboardService>().GetOrderFiguresAsync();
        return new TwinOrders(o.Queued, o.Active, o.Completed, o.AverageLeadTimeSeconds, o.LastHour);
    }

    private async Task<Reply> Run<T>(double t, Func<IServiceProvider, Task<T>> command)
    {
        clock.FollowSimulation(t);
        try
        {
            var (ok, error, value) = await wms.TryCommandAsync(command);
            return new Reply(ok, error, value);
        }
        catch (Exception ex)
        {
            return new Reply(false, ex.Message, null);
        }
    }
}
