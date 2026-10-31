using Logistics.Wms.Application.Abstractions;
using Logistics.Wms.Domain;
using Microsoft.EntityFrameworkCore;

namespace Logistics.Wms.Application.Services;

/// <summary>
/// Availability and task timing of the five robots, used by the process services: which robots may get work, and when a robot starts
/// and finishes a task (cycle time, busy time). Saves nothing; the calling service saves with its own changes.
/// </summary>
public sealed class RobotTracker(IWmsDbContext db, TimeProvider clock)
{
    public async Task<Robot> GetAsync(string id, CancellationToken ct = default) =>
        await db.Robots.SingleOrDefaultAsync(r => r.Id == id, ct) ?? throw WmsException.NotFound("Robot", id);

    public async Task<bool> IsAvailableAsync(string id, CancellationToken ct = default) =>
        await db.Robots.AnyAsync(r => r.Id == id && r.Mode == RobotMode.Auto && r.State != RobotState.Fault, ct);

    /// <summary>Sorters that must not get work, with the lanes they serve.</summary>
    public async Task<(HashSet<Sorter> Sorters, List<string> Lanes)> UnavailableSortersAsync(CancellationToken ct = default)
    {
        var down = (await db.Robots.Where(r => r.Role == RobotRole.Sorter && (r.Mode != RobotMode.Auto || r.State == RobotState.Fault))
            .Select(r => r.Sorter).ToListAsync(ct)).Where(s => s != null).Select(s => s!.Value).ToHashSet();
        return (down, [.. down.SelectMany(CellLayout.LaneIdsOf)]);
    }

    /// <summary>The robot starts a task. Starting the task it already works on changes nothing; another running task is finished first.</summary>
    public async Task BeginAsync(string robotId, string taskRef, string text, CancellationToken ct = default)
    {
        var r = await GetAsync(robotId, ct);
        if (r.TaskRef == taskRef) return;
        if (r.TaskRef is not null) Finish(r);
        r.TaskRef = taskRef;
        r.TaskText = text;
        r.TaskStartedAt = clock.GetUtcNow().UtcDateTime;
        if (r.State != RobotState.Fault) r.State = RobotState.Working;
    }

    /// <summary>The robot finished its task (any task when <paramref name="taskRef"/> is null): one cycle, its time counts as busy.</summary>
    public async Task EndAsync(string robotId, string? taskRef = null, CancellationToken ct = default)
    {
        var r = await GetAsync(robotId, ct);
        if (r.TaskRef is null || (taskRef is not null && r.TaskRef != taskRef)) return;
        Finish(r);
    }

    /// <summary>The task was dropped (cancelled, carton missed): no cycle is counted.</summary>
    public async Task AbortAsync(string robotId, string? taskRef = null, CancellationToken ct = default)
    {
        var r = await GetAsync(robotId, ct);
        if (r.TaskRef is null || (taskRef is not null && r.TaskRef != taskRef)) return;
        Clear(r);
    }

    private void Finish(Robot r)
    {
        var seconds = r.TaskStartedAt is DateTime t0 ? Math.Max(0, (clock.GetUtcNow().UtcDateTime - t0).TotalSeconds) : 0;
        r.Cycles++;
        r.BusySeconds += seconds;
        r.LastCycleSeconds = seconds;
        Clear(r);
    }

    private static void Clear(Robot r)
    {
        r.TaskRef = null;
        r.TaskText = null;
        r.TaskStartedAt = null;
        if (r.State != RobotState.Fault) r.State = RobotState.Idle;
    }
}
