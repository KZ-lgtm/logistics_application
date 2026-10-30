using Logistics.Wms.Application.Abstractions;
using Logistics.Wms.Application.Contracts;
using Logistics.Wms.Domain;
using Microsoft.EntityFrameworkCore;

namespace Logistics.Wms.Application.Services;

/// <summary>
/// Task dispatch for R1 in the cylinder shelf. R1 executes one task at a time; the next task is chosen in this order:
/// <list type="number">
/// <item>a bin back at the picking-loop return head (empty → restock loop, else store: it blocks the loop),</item>
/// <item>a refilled bin at the restock-loop return head (store),</item>
/// <item>a retrieval for uncovered wave demand while the picking loop has room,</item>
/// <item>an empty bin from the rack to the restock loop.</item>
/// </list>
/// </summary>
public sealed class AsrsService(IWmsDbContext db, DemandService demand, TimeProvider clock)
{
    public async Task<AsrsTaskDto?> GetCurrentTaskAsync(CancellationToken ct = default)
    {
        var task = await db.AsrsTasks.Where(t => t.Status == AsrsTaskStatus.Assigned).OrderBy(t => t.Id).FirstOrDefaultAsync(ct);
        return task is null ? null : await ToDtoAsync(task, ct);
    }

    public async Task<IReadOnlyList<AsrsTaskDto>> ListAsync(int take, CancellationToken ct = default)
    {
        var tasks = await db.AsrsTasks.OrderByDescending(t => t.Id).Take(Math.Clamp(take, 1, 500)).ToListAsync(ct);
        var result = new List<AsrsTaskDto>(tasks.Count);
        foreach (var t in tasks) result.Add(await ToDtoAsync(t, ct));
        return result;
    }

    /// <summary>Returns R1's assigned task, or assigns the next one; null when R1 has nothing to do.</summary>
    public async Task<AsrsTaskDto?> RequestNextTaskAsync(CancellationToken ct = default)
    {
        var current = await GetCurrentTaskAsync(ct);
        if (current is not null) return current;

        var task = await DecideAsync(ct);
        if (task is null) return null;

        var bin = await db.Bins.SingleAsync(b => b.Id == task.BinId, ct);
        bin.Reserved = true;
        if (task.ToSlotId is int to) (await db.Slots.SingleAsync(s => s.Id == to, ct)).Reserved = true;
        task.Status = AsrsTaskStatus.Assigned;
        task.CreatedAt = clock.GetUtcNow().UtcDateTime;
        db.AsrsTasks.Add(task);
        await db.SaveChangesAsync(ct);
        return await ToDtoAsync(task, ct);
    }

    /// <summary>R1 reports the task as done: the bin is at its destination.</summary>
    public async Task<AsrsTaskDto> CompleteTaskAsync(int taskId, CancellationToken ct = default)
    {
        var task = await db.AsrsTasks.SingleOrDefaultAsync(t => t.Id == taskId, ct) ?? throw WmsException.NotFound("ASRS task", taskId);
        if (task.Status != AsrsTaskStatus.Assigned) throw WmsException.Conflict($"ASRS task {taskId} is {task.Status}.");

        var now = clock.GetUtcNow().UtcDateTime;
        var bin = await db.Bins.SingleAsync(b => b.Id == task.BinId, ct);
        var state = await db.CellState.SingleAsync(ct);

        if (task.FromSlotId is int from)
        {
            var slot = await db.Slots.SingleAsync(s => s.Id == from, ct);
            slot.BinId = null;
            bin.SlotId = null;
            state.R1Angle = slot.Angle;
        }

        switch (task.Kind)
        {
            case AsrsTaskKind.Retrieve:
                bin.Location = BinLocation.PickInbound;
                bin.Station = null;
                bin.Allocated = false;
                state.R1Angle = 0;                                      // east port
                break;
            case AsrsTaskKind.ReturnToRestock:
            case AsrsTaskKind.RackToRestock:
                bin.Location = BinLocation.RestockStation;
                state.R1Angle = Math.PI;                                // west port
                break;
            case AsrsTaskKind.Store:
            case AsrsTaskKind.StoreRestocked:
                var slot = await db.Slots.SingleAsync(s => s.Id == task.ToSlotId, ct);
                slot.BinId = bin.Id;
                slot.Reserved = false;
                bin.SlotId = slot.Id;
                bin.Location = BinLocation.Rack;
                bin.Station = null;
                bin.Allocated = false;
                state.R1Angle = slot.Angle;
                break;
        }

        bin.Reserved = false;
        bin.LastMovedAt = now;
        task.Status = AsrsTaskStatus.Completed;
        task.CompletedAt = now;
        await db.SaveChangesAsync(ct);
        return await ToDtoAsync(task, ct);
    }

    /// <summary>R1 could not execute the task (e.g. bin not found by the fork sensor): reservations are released.</summary>
    public async Task<AsrsTaskDto> CancelTaskAsync(int taskId, CancellationToken ct = default)
    {
        var task = await db.AsrsTasks.SingleOrDefaultAsync(t => t.Id == taskId, ct) ?? throw WmsException.NotFound("ASRS task", taskId);
        if (task.Status != AsrsTaskStatus.Assigned) throw WmsException.Conflict($"ASRS task {taskId} is {task.Status}.");
        (await db.Bins.SingleAsync(b => b.Id == task.BinId, ct)).Reserved = false;
        if (task.ToSlotId is int to) (await db.Slots.SingleAsync(s => s.Id == to, ct)).Reserved = false;
        task.Status = AsrsTaskStatus.Cancelled;
        task.CompletedAt = clock.GetUtcNow().UtcDateTime;
        await db.SaveChangesAsync(ct);
        return await ToDtoAsync(task, ct);
    }

    private async Task<AsrsTask?> DecideAsync(CancellationToken ct)
    {
        var angle = (await db.CellState.SingleAsync(ct)).R1Angle;
        var restockCount = await db.Bins.CountAsync(b => b.Location == BinLocation.RestockStation || b.Location == BinLocation.RestockReturn, ct);

        // 1. picking-loop return head
        var back = await db.Bins.Where(b => b.Location == BinLocation.PickReturn && !b.Reserved)
            .OrderBy(b => b.LastMovedAt).ThenBy(b => b.Id).FirstOrDefaultAsync(ct);
        if (back is not null)
        {
            if (back.IsEmpty && restockCount < CellLayout.RestockLoopCapacity)
                return new AsrsTask { Kind = AsrsTaskKind.ReturnToRestock, BinId = back.Id };
            if (await FreeSlotAsync(angle, ct) is { } slot)
                return new AsrsTask { Kind = AsrsTaskKind.Store, BinId = back.Id, ToSlotId = slot.Id };
        }

        // 2. restock-loop return head
        var refilled = await db.Bins.Where(b => b.Location == BinLocation.RestockReturn && !b.Reserved)
            .OrderBy(b => b.LastMovedAt).ThenBy(b => b.Id).FirstOrDefaultAsync(ct);
        if (refilled is not null && await FreeSlotAsync(angle, ct) is { } restockSlot)
            return new AsrsTask { Kind = AsrsTaskKind.StoreRestocked, BinId = refilled.Id, ToSlotId = restockSlot.Id };

        // 3. retrieval for wave demand
        var onPickLoop = await db.Bins.CountAsync(b =>
            b.Location == BinLocation.PickInbound || b.Location == BinLocation.AtStation || b.Location == BinLocation.PickReturn, ct);
        if (onPickLoop < CellLayout.PickLoopCapacity && await demand.NextNeededSkuAsync(ct) is int sku)
        {
            var candidates = await db.Bins.Include(b => b.Slot)
                .Where(b => b.SkuId == sku && b.Location == BinLocation.Rack && b.Quantity > 0 && !b.Reserved)
                .ToListAsync(ct);
            if (SlottingPolicy.ChooseBin(candidates, angle) is { } bin)
                return new AsrsTask { Kind = AsrsTaskKind.Retrieve, BinId = bin.Id, FromSlotId = bin.SlotId, ForSkuId = sku };
        }

        // 4. empty bin in the rack → restock
        if (restockCount < CellLayout.RestockLoopRackLimit)
        {
            var empty = await db.Bins.Where(b => b.Location == BinLocation.Rack && b.Quantity <= 0 && !b.Reserved)
                .OrderBy(b => b.Id).FirstOrDefaultAsync(ct);
            if (empty is not null) return new AsrsTask { Kind = AsrsTaskKind.RackToRestock, BinId = empty.Id, FromSlotId = empty.SlotId };
        }
        return null;
    }

    private async Task<StorageSlot?> FreeSlotAsync(double angle, CancellationToken ct) =>
        SlottingPolicy.ChooseFreeSlot(await db.Slots.Where(s => s.BinId == null && !s.Reserved && !s.Blocked).ToListAsync(ct), angle);

    private async Task<AsrsTaskDto> ToDtoAsync(AsrsTask t, CancellationToken ct)
    {
        var bin = await db.Bins.Include(b => b.Sku).SingleAsync(b => b.Id == t.BinId, ct);
        var from = t.FromSlotId is int f ? (await db.Slots.SingleAsync(s => s.Id == f, ct)).Code : null;
        var to = t.ToSlotId is int d ? (await db.Slots.SingleAsync(s => s.Id == d, ct)).Code : null;
        return new AsrsTaskDto(t.Id, t.Kind, t.Status, t.BinId, bin.Sku.Code, bin.Quantity, from, to,
            t.Kind.Describe(true, from), t.Kind.Describe(false, to), t.CreatedAt, t.CompletedAt);
    }
}
