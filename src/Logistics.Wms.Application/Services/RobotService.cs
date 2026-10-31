using Logistics.Wms.Application.Abstractions;
using Logistics.Wms.Application.Contracts;
using Logistics.Wms.Domain;
using Microsoft.EntityFrameworkCore;

namespace Logistics.Wms.Application.Services;

/// <summary>
/// The five robots as the WMS sees them: mode and faults (set by people or reported by the controllers), the task each one works on,
/// lifetime figures, and robot-centric dispatch (each controller asks "what next?" and reports "done").
/// <para>
/// When a robot is down the WMS works around it: R1 gets no new task; a picker's stop is skipped at the decision point and the bin
/// standing there is released with its open picks back to demand (the other stop takes over); a sorter's lanes take no new order and
/// its orders get no new picks until it is back.
/// </para>
/// </summary>
public sealed class RobotService(
    IWmsDbContext db, RobotTracker tracker, AsrsService asrs, PickingService picking, SortationService sortation, PutWallService putWall, TimeProvider clock)
{
    public async Task<IReadOnlyList<RobotDto>> ListAsync(CancellationToken ct = default)
    {
        var now = clock.GetUtcNow().UtcDateTime;
        return [.. (await db.Robots.ToListAsync(ct)).OrderBy(r => r.Role).ThenBy(r => r.Id).Select(r => ToDto(r, now))];
    }

    public async Task<RobotDto> GetAsync(string id, CancellationToken ct = default) => ToDto(await tracker.GetAsync(Normalize(id), ct), clock.GetUtcNow().UtcDateTime);

    public async Task<IReadOnlyList<RobotEventDto>> ListEventsAsync(string? robotId, int take, CancellationToken ct = default)
    {
        var q = db.RobotEvents.AsQueryable();
        if (!string.IsNullOrWhiteSpace(robotId)) { var id = Normalize(robotId); q = q.Where(e => e.RobotId == id); }
        return await q.OrderByDescending(e => e.Id).Take(Math.Clamp(take, 1, 500))
            .Select(e => new RobotEventDto(e.Id, e.RobotId, e.At, e.Kind, e.Code, e.Text, e.User)).ToListAsync(ct);
    }

    public async Task<RobotDto> SetModeAsync(string id, SetRobotModeRequest request, CancellationToken ct = default)
    {
        var r = await tracker.GetAsync(Normalize(id), ct);
        if (r.Mode == request.Mode) return ToDto(r, clock.GetUtcNow().UtcDateTime);
        var was = r.Mode;
        r.Mode = request.Mode;
        Log(r, RobotEventKind.ModeChanged, null, $"mode {was} → {request.Mode}", request.User);
        await AfterAvailabilityChangeAsync(r, ct);
        await db.SaveChangesAsync(ct);
        return ToDto(r, clock.GetUtcNow().UtcDateTime);
    }

    /// <summary>The controller (or a person) reports a fault: the robot gets no work until the fault is cleared.</summary>
    public async Task<RobotDto> ReportFaultAsync(string id, RobotFaultRequest request, CancellationToken ct = default)
    {
        if (string.IsNullOrWhiteSpace(request.Code)) throw WmsException.Invalid("A fault code is required.");
        var r = await tracker.GetAsync(Normalize(id), ct);
        var now = clock.GetUtcNow().UtcDateTime;
        if (r.State == RobotState.Fault) CloseFaultTime(r, now);   // a new fault replaces the old one
        else r.Faults++;
        r.State = RobotState.Fault;
        r.FaultCode = request.Code.Trim();
        r.FaultText = string.IsNullOrWhiteSpace(request.Text) ? null : request.Text.Trim();
        r.FaultSince = now;
        Log(r, RobotEventKind.Fault, r.FaultCode, r.FaultText ?? "fault", request.User);
        await AfterAvailabilityChangeAsync(r, ct);
        await db.SaveChangesAsync(ct);
        return ToDto(r, now);
    }

    public async Task<RobotDto> ClearFaultAsync(string id, string? user = null, CancellationToken ct = default)
    {
        var r = await tracker.GetAsync(Normalize(id), ct);
        if (r.State != RobotState.Fault) throw WmsException.Conflict($"{r.Id} has no fault.");
        var now = clock.GetUtcNow().UtcDateTime;
        CloseFaultTime(r, now);
        Log(r, RobotEventKind.FaultCleared, r.FaultCode, $"fault {r.FaultCode} cleared", user);
        r.State = r.TaskRef is null ? RobotState.Idle : RobotState.Working;
        r.FaultCode = null;
        r.FaultText = null;
        r.FaultSince = null;
        await AfterAvailabilityChangeAsync(r, ct);
        await db.SaveChangesAsync(ct);
        return ToDto(r, now);
    }

    public async Task ResetStatisticsAsync(string? user = null, CancellationToken ct = default)
    {
        var now = clock.GetUtcNow().UtcDateTime;
        foreach (var r in await db.Robots.ToListAsync(ct))
        {
            r.Cycles = 0; r.BusySeconds = 0; r.LastCycleSeconds = 0; r.FaultSeconds = 0; r.Faults = r.State == RobotState.Fault ? 1 : 0;
            r.StatsSince = now;
            if (r.FaultSince is not null) r.FaultSince = now;
            if (r.TaskStartedAt is not null) r.TaskStartedAt = now;
            Log(r, RobotEventKind.StatsReset, null, "figures reset", user);
        }
        await db.SaveChangesAsync(ct);
    }

    // ---------------------------------------------------------------- robot-centric dispatch

    /// <summary>
    /// The robot's controller asks for work: R1 gets its ASRS task, a picker the next pick of the bin at its stop, a sorter the oldest
    /// carton routed to it. Null when there is nothing to do or the robot is not available. Asking again returns the same task.
    /// </summary>
    public async Task<RobotTaskDto?> NextTaskAsync(string id, AsrsHeads? heads = null, CancellationToken ct = default)
    {
        var r = await tracker.GetAsync(Normalize(id), ct);
        switch (r.Role)
        {
            case RobotRole.ShelfRobot:
                var t = await asrs.RequestNextTaskAsync(heads, ct);
                return t is null ? null : new RobotTaskDto(r.Id, t.Kind.ToString(), t.Id.ToString(), $"{t.Kind.Describe()} bin {t.BinId} ({t.SkuCode}, {t.BinQuantity} pcs)",
                    t.BinId, t.SkuCode, null, null, t.From, t.To);

            case RobotRole.Picker:
                if (!r.IsAvailable) return null;
                var st = await picking.GetStationAsync(r.Station!.Value, ct);
                if (st.Picks.Count == 0) return null;
                var p = st.Picks[0];
                await tracker.BeginAsync(r.Id, $"pick:{p.PickTaskId}", $"pick {st.SkuCode} from bin {st.BinId} for order {p.OrderId}", ct);
                await db.SaveChangesAsync(ct);
                return new RobotTaskDto(r.Id, "Pick", p.PickTaskId.ToString(), $"pick {st.SkuCode} from bin {st.BinId} for order {p.OrderId} ({p.OrderReference})",
                    st.BinId, st.SkuCode, p.OrderId, null, $"bin {st.BinId} at {st.Station}", "item belt");

            default:
                if (!r.IsAvailable) return null;
                var c = await (from pt in db.PickTasks
                               join s in db.Skus on pt.SkuId equals s.Id
                               where pt.Status == PickStatus.Routed && pt.TargetSorter == r.Sorter
                               orderby pt.ScannedAt, pt.Id
                               select new { pt.Id, pt.OrderId, pt.TargetLaneId, s.Code }).FirstOrDefaultAsync(ct);
                if (c is null) return null;
                await tracker.BeginAsync(r.Id, $"sort:{c.Id}", $"put {c.Code} (carton #{c.Id}) into lane {c.TargetLaneId}", ct);
                await db.SaveChangesAsync(ct);
                return new RobotTaskDto(r.Id, "Sort", c.Id.ToString(), $"put {c.Code} (carton #{c.Id}) into lane {c.TargetLaneId} for order {c.OrderId}",
                    null, c.Code, c.OrderId, c.TargetLaneId, "belt stop", $"lane {c.TargetLaneId}");
        }
    }

    /// <summary>The robot reports its task done (an ASRS task, a pick placed on the belt, a carton put into its case).</summary>
    public async Task<RobotTaskDto> CompleteTaskAsync(string id, string taskRef, CancellationToken ct = default)
    {
        var r = await tracker.GetAsync(Normalize(id), ct);
        switch (r.Role)
        {
            case RobotRole.ShelfRobot:
                if (!int.TryParse(taskRef, out var asrsId)) throw WmsException.Invalid($"'{taskRef}' is not an ASRS task id.");
                var t = await asrs.CompleteTaskAsync(asrsId, ct);
                return new RobotTaskDto(r.Id, t.Kind.ToString(), t.Id.ToString(), $"{t.Kind.Describe()} bin {t.BinId} done", t.BinId, t.SkuCode, null, null, t.From, t.To);

            case RobotRole.Picker:
                var pickId = ParsePick(taskRef);
                var station = await db.PickTasks.Where(p => p.Id == pickId).Select(p => (PickStation?)p.Station).SingleOrDefaultAsync(ct)
                    ?? throw WmsException.NotFound("Pick task", pickId);
                if (station != r.Station) throw WmsException.Conflict($"Pick task {pickId} belongs to stop {station}, not to {r.Id} at {r.Station}.");
                var done = await picking.ConfirmPickAsync(pickId, ct);
                return new RobotTaskDto(r.Id, "Pick", done.PickTaskId.ToString(), $"carton #{done.PickTaskId} on the belt", done.BinId, null, null, null, null, "item belt");

            default:
                var sortId = ParsePick(taskRef);
                var target = await db.PickTasks.Where(p => p.Id == sortId).Select(p => new { p.TargetSorter, p.TargetLaneId }).SingleOrDefaultAsync(ct)
                    ?? throw WmsException.NotFound("Pick task", sortId);
                if (target.TargetSorter != r.Sorter || target.TargetLaneId is null) throw WmsException.Conflict($"Carton #{sortId} is not routed to {r.Id}.");
                var sorted = await sortation.ConfirmSortAsync(new SortConfirmRequest(sortId, target.TargetLaneId), ct);
                return new RobotTaskDto(r.Id, "Sort", sorted.PickTaskId.ToString(),
                    sorted.OrderCompleted ? $"order {sorted.OrderId} complete" : $"carton #{sorted.PickTaskId} in lane {sorted.LaneId}",
                    null, null, sorted.OrderId, sorted.LaneId, null, $"lane {sorted.LaneId}");
        }
    }

    // ---------------------------------------------------------------- helpers

    private async Task AfterAvailabilityChangeAsync(Robot r, CancellationToken ct)
    {
        if (r.IsAvailable)
        {
            if (r.Role == RobotRole.Sorter) { await db.SaveChangesAsync(ct); await putWall.ReleaseOrdersAsync(ct); }
            return;
        }
        if (r.Role == RobotRole.Picker) await ReleasePickerWorkAsync(r, ct);
    }

    /// <summary>A picker went down: the bin at its stop leaves with its open picks back to demand; bins heading there are re-routed.</summary>
    private async Task ReleasePickerWorkAsync(Robot r, CancellationToken ct)
    {
        var station = r.Station!.Value;
        var now = clock.GetUtcNow().UtcDateTime;
        var bin = await db.Bins.FirstOrDefaultAsync(b => b.Location == BinLocation.AtStation && b.Station == station, ct);
        if (bin is not null)
        {
            var pending = await db.PickTasks.Include(p => p.OrderLine)
                .Where(p => p.BinId == bin.Id && p.Status == PickStatus.Pending).ToListAsync(ct);
            foreach (var p in pending)
            {
                p.Status = PickStatus.Cancelled;
                p.OrderLine.Allocated--;
            }
            PickingService.MoveToReturn(bin, now);
            Log(r, RobotEventKind.WorkReassigned, null, $"bin {bin.Id} released from {station}: {pending.Count} open picks back to demand", null);
        }
        foreach (var inbound in await db.Bins.Where(b => b.Location == BinLocation.PickInbound && b.Station == station).ToListAsync(ct))
        {
            inbound.Station = null;
            Log(r, RobotEventKind.WorkReassigned, null, $"bin {inbound.Id} re-routed at the decision point", null);
        }
        if (r.TaskRef is not null) await tracker.AbortAsync(r.Id, null, ct);
    }

    private void Log(Robot r, RobotEventKind kind, string? code, string text, string? user) =>
        db.RobotEvents.Add(new RobotEvent { RobotId = r.Id, At = clock.GetUtcNow().UtcDateTime, Kind = kind, Code = code, Text = text, User = user });

    private static void CloseFaultTime(Robot r, DateTime now)
    {
        if (r.FaultSince is DateTime since) r.FaultSeconds += Math.Max(0, (now - since).TotalSeconds);
    }

    private static long ParsePick(string taskRef) =>
        long.TryParse(taskRef, out var id) ? id : throw WmsException.Invalid($"'{taskRef}' is not a carton (pick task) id.");

    private static string Normalize(string id) => id.Trim().ToUpperInvariant();

    internal static RobotDto ToDto(Robot r, DateTime now)
    {
        var elapsed = Math.Max(1, (now - r.StatsSince).TotalSeconds);
        var running = r.TaskStartedAt is DateTime t0 ? Math.Max(0, (now - t0).TotalSeconds) : 0;
        var faultNow = r.FaultSince is DateTime f ? Math.Max(0, (now - f).TotalSeconds) : 0;
        var assignment = r.Role switch
        {
            RobotRole.ShelfRobot => "cylinder shelf ↔ picking / restock loop",
            RobotRole.Picker => $"picking stop {r.Station}",
            _ => $"lanes {r.Sorter}-1…4",
        };
        return new RobotDto(r.Id, r.Model, r.Role, assignment, r.Tool, r.Controller, r.Mode, r.State, r.IsAvailable, r.FaultCode, r.FaultText, r.FaultSince,
            r.TaskRef, r.TaskText, r.TaskStartedAt, r.Cycles, r.Cycles == 0 ? null : r.BusySeconds / r.Cycles, r.LastCycleSeconds,
            Math.Clamp((r.BusySeconds + running) / elapsed, 0, 1), Math.Clamp(1 - (r.FaultSeconds + faultNow) / elapsed, 0, 1), r.Faults, r.StatsSince);
    }
}
