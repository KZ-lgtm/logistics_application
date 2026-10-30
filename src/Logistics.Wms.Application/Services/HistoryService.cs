using Logistics.Wms.Application.Abstractions;
using Logistics.Wms.Domain;
using Microsoft.EntityFrameworkCore;

namespace Logistics.Wms.Application.Services;

public sealed record CompactionResult(int OrdersRemoved, int PickTasksRemoved, int AsrsTasksRemoved, int TransactionsFolded, int OpeningBalances);

/// <summary>
/// Keeps the operational database small: finished orders beyond the newest <c>keepOrders</c> are removed with their lines, pick
/// tasks and resolved exceptions; old ASRS tasks are dropped; ledger entries beyond the newest <c>keepTransactions</c> are folded into
/// one opening balance per bin (or per SKU restock buffer), so every stock total stays exact. Lifetime KPIs are carried in
/// <see cref="CellState"/>.
/// </summary>
public sealed class HistoryService(IWmsDbContext db)
{
    public async Task<CompactionResult> CompactAsync(int keepOrders = 300, int keepAsrsTasks = 300, int keepTransactions = 2000, CancellationToken ct = default)
    {
        var state = await db.CellState.SingleAsync(ct);

        // finished orders
        var old = await db.Orders.Where(o => o.Status == OrderStatus.Completed || o.Status == OrderStatus.Cancelled)
            .OrderByDescending(o => o.Id).Skip(Math.Max(0, keepOrders)).Select(o => new { o.Id, o.Status }).ToListAsync(ct);
        var ids = old.Select(o => o.Id).ToList();
        var picks = await db.PickTasks.Where(p => ids.Contains(p.OrderId)).ToListAsync(ct);
        var pickIds = picks.Select(p => p.Id).ToList();
        var exceptions = await db.ExceptionItems.Where(e => pickIds.Contains(e.PickTaskId) && e.Disposition != ExceptionDisposition.Open).ToListAsync(ct);
        var openException = (await db.ExceptionItems.Where(e => pickIds.Contains(e.PickTaskId) && e.Disposition == ExceptionDisposition.Open)
            .Select(e => e.OrderId).ToListAsync(ct)).ToHashSet();
        // an order with a still open exception stays until the exception is resolved
        ids.RemoveAll(openException.Contains);
        picks.RemoveAll(p => openException.Contains(p.OrderId));
        exceptions.RemoveAll(e => openException.Contains(e.OrderId));

        state.ArchivedOrdersCompleted += old.Count(o => o.Status == OrderStatus.Completed && ids.Contains(o.Id));
        state.ArchivedOrdersCancelled += old.Count(o => o.Status == OrderStatus.Cancelled && ids.Contains(o.Id));
        state.ArchivedItemsPicked += picks.Count(p => p.PickedAt != null);
        state.ArchivedItemsSorted += picks.Count(p => p.Status == PickStatus.Sorted);
        state.ArchivedScans += picks.Count(p => p.ScannedAt != null);
        state.ArchivedNoReads += exceptions.Count(e => e.Reason == ExceptionReason.NoRead);

        db.ExceptionItems.RemoveRange(exceptions);
        db.PickTasks.RemoveRange(picks);
        db.OrderLines.RemoveRange(await db.OrderLines.Where(l => ids.Contains(l.OrderId)).ToListAsync(ct));
        db.Orders.RemoveRange(await db.Orders.Where(o => ids.Contains(o.Id)).ToListAsync(ct));

        // ASRS history
        var asrs = await db.AsrsTasks.Where(t => t.Status != AsrsTaskStatus.Assigned)
            .OrderByDescending(t => t.Id).Skip(Math.Max(0, keepAsrsTasks)).ToListAsync(ct);
        db.AsrsTasks.RemoveRange(asrs);

        // ledger → opening balances
        var folded = await db.InventoryTransactions.OrderByDescending(t => t.Id).Skip(Math.Max(0, keepTransactions)).ToListAsync(ct);
        var balances = folded.GroupBy(t => (t.SkuId, t.BinId))
            .Select(g => new InventoryTransaction
            {
                At = g.Max(t => t.At), Type = InventoryTransactionType.OpeningBalance, SkuId = g.Key.SkuId, BinId = g.Key.BinId,
                Quantity = g.Sum(t => t.Quantity), Reference = $"opening balance of {g.Count()} bookings up to {g.Max(t => t.At):u}",
            })
            .Where(t => t.Quantity != 0).ToList();
        db.InventoryTransactions.RemoveRange(folded);
        db.InventoryTransactions.AddRange(balances);

        await db.SaveChangesAsync(ct);
        return new CompactionResult(ids.Count, picks.Count, asrs.Count, folded.Count, balances.Count);
    }
}
