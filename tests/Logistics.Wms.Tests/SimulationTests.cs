using Logistics.Wms.Application.Services;
using Logistics.Wms.Application.Simulation;
using Logistics.Wms.Domain;
using Logistics.Wms.Persistence;
using Microsoft.EntityFrameworkCore;

namespace Logistics.Wms.Tests;

/// <summary>Long runs of the cell simulator against the real services; checks the invariants the 3D simulation verifies.</summary>
public class SimulationTests
{
    [Theory]
    [InlineData(true, 0.005, false)]
    [InlineData(false, 0.05, false)]
    [InlineData(true, 0.02, true)]     // EF Core in-memory provider, as in the browser edition
    public async Task Long_run_keeps_stock_bins_lanes_and_orders_consistent(bool pareto, double noRead, bool inMemory)
    {
        await using var h = await WmsHarness.CreateAsync(new SimulationOptions { ParetoDemand = pareto, NoReadRate = noRead, Seed = 11 }, inMemory);
        await h.RunSimulationAsync(1500);

        var orders = await h.Db.Orders.CountAsync(o => o.Status == OrderStatus.Completed);
        Assert.True(orders > 300, $"throughput: {orders} orders");
        if (noRead > 0.01) Assert.True(await h.Db.ExceptionItems.AnyAsync(e => e.Reason == ExceptionReason.NoRead));
        await AssertConsistentAsync(h.Db, ledgerHasAllPicks: true);
    }

    [Fact]
    public async Task Snapshot_restores_the_complete_state_and_the_cell_carries_on()
    {
        await using var a = await WmsHarness.CreateAsync(new SimulationOptions { NoReadRate = 0.03, Seed = 5 }, inMemory: true);
        await a.RunSimulationAsync(300);
        var json = (await WmsSnapshot.CaptureAsync(a.Db, a.Clock.GetUtcNow().UtcDateTime)).ToJson();

        await using var b = await WmsHarness.CreateAsync(new SimulationOptions { NoReadRate = 0.03, Seed = 6 }, inMemory: true, seed: false);
        await WmsSnapshot.FromJson(json).RestoreAsync(b.Db);
        await b.NewScopeAsync();

        Assert.Equal(await a.Db.Orders.CountAsync(), await b.Db.Orders.CountAsync());
        Assert.Equal(await a.Db.PickTasks.CountAsync(), await b.Db.PickTasks.CountAsync());
        Assert.Equal(await a.Db.InventoryTransactions.SumAsync(t => t.Quantity), await b.Db.InventoryTransactions.SumAsync(t => t.Quantity));
        Assert.Equal(
            await a.Db.Bins.OrderBy(x => x.Id).Select(x => new { x.Id, x.Quantity, x.Location, x.SlotId }).ToListAsync(),
            await b.Db.Bins.OrderBy(x => x.Id).Select(x => new { x.Id, x.Quantity, x.Location, x.SlotId }).ToListAsync());

        // new rows get fresh keys after the explicitly keyed restore
        var maxOrder = await b.Db.Orders.MaxAsync(o => o.Id);
        await b.RunSimulationAsync(300);
        Assert.True(await b.Db.Orders.MaxAsync(o => o.Id) > maxOrder);
        await AssertConsistentAsync(b.Db, ledgerHasAllPicks: true);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task Compaction_shrinks_history_but_keeps_stock_and_lifetime_figures(bool inMemory)
    {
        await using var h = await WmsHarness.CreateAsync(new SimulationOptions { NoReadRate = 0.03, Seed = 3 }, inMemory);
        await h.RunSimulationAsync(600);
        var before = await h.Get<DashboardService>().GetAsync();
        var stock = await StockBySkuAsync(h.Db);

        var result = await h.Get<HistoryService>().CompactAsync(keepOrders: 20, keepAsrsTasks: 20, keepTransactions: 100);
        await h.NewScopeAsync();
        Assert.True(result.OrdersRemoved > 50);
        Assert.True(result.TransactionsFolded > 0);
        Assert.True(await h.Db.Orders.CountAsync() < 60);

        var after = await h.Get<DashboardService>().GetAsync();
        Assert.Equal(before.Orders.Completed, after.Orders.Completed);
        Assert.Equal(before.Throughput.ItemsSortedTotal, after.Throughput.ItemsSortedTotal);
        Assert.Equal(before.Throughput.ItemsPickedTotal, after.Throughput.ItemsPickedTotal);
        Assert.Equal(before.Throughput.NoReads, after.Throughput.NoReads);
        Assert.Equal(stock, await StockBySkuAsync(h.Db));
        await AssertConsistentAsync(h.Db, ledgerHasAllPicks: false);

        await h.RunSimulationAsync(300);
        await AssertConsistentAsync(h.Db, ledgerHasAllPicks: false);
    }

    private static async Task<Dictionary<int, int>> StockBySkuAsync(WmsDbContext db) =>
        await db.InventoryTransactions.GroupBy(t => t.SkuId).Select(g => new { g.Key, Q = g.Sum(t => t.Quantity) }).ToDictionaryAsync(x => x.Key, x => x.Q);

    internal static async Task AssertConsistentAsync(WmsDbContext db, bool ledgerHasAllPicks)
    {
        var orders = await db.Orders.Include(o => o.Lines).ToListAsync();
        Assert.All(orders.Where(o => o.Status == OrderStatus.Completed), o => Assert.All(o.Lines, l => Assert.Equal(l.Quantity, l.Sorted)));
        Assert.All(orders.SelectMany(o => o.Lines), l =>
        {
            Assert.InRange(l.Sorted, 0, l.Quantity);
            Assert.InRange(l.Picked, l.Sorted, l.Allocated);
            Assert.InRange(l.Allocated, 0, l.Quantity);
        });

        // stock ledger = bins + exception buffer
        var bins = await db.Bins.Include(b => b.Sku).ToListAsync();
        foreach (var sku in await db.Skus.ToListAsync())
        {
            var ledger = await db.InventoryTransactions.Where(t => t.SkuId == sku.Id).SumAsync(t => t.Quantity);
            Assert.Equal(ledger, bins.Where(b => b.SkuId == sku.Id).Sum(b => b.Quantity) + sku.RestockBuffer);
        }
        Assert.All(bins, b => Assert.InRange(b.Quantity, 0, b.Sku.BinCapacity));

        // cartons conserved: every carton picked is on the belt, in a case or went to the exception tote
        var picks = await db.PickTasks.ToListAsync();
        var picked = picks.Count(p => p.PickedAt != null);
        Assert.Equal(picked, picks.Count(p => p.Status is PickStatus.OnBelt or PickStatus.Routed or PickStatus.Sorted or PickStatus.Exception));
        Assert.Equal(picks.Count(p => p.Status == PickStatus.Exception), await db.ExceptionItems.CountAsync());
        if (ledgerHasAllPicks)
            Assert.Equal(-picked, await db.InventoryTransactions.Where(t => t.Type == InventoryTransactionType.Pick).SumAsync(t => t.Quantity));

        // slots ↔ bins
        var slots = await db.Slots.ToListAsync();
        Assert.All(bins.Where(b => b.Location == BinLocation.Rack), b => Assert.Equal(b.Id, slots.Single(s => s.Id == b.SlotId).BinId));
        Assert.All(bins.Where(b => b.Location != BinLocation.Rack), b => Assert.Null(b.SlotId));
        Assert.Equal(slots.Count(s => s.BinId != null), bins.Count(b => b.Location == BinLocation.Rack));
        Assert.Equal(slots.Count(s => s.BinId != null), slots.Where(s => s.BinId != null).Select(s => s.BinId).Distinct().Count());
        Assert.True(bins.Count(b => b.Location is BinLocation.PickInbound or BinLocation.AtStation or BinLocation.PickReturn) <= CellLayout.PickLoopCapacity);
        Assert.True(bins.Count(b => b.Location is BinLocation.RestockStation or BinLocation.RestockReturn) <= CellLayout.RestockLoopCapacity);
        foreach (var st in Enum.GetValues<PickStation>()) Assert.True(bins.Count(b => b.Location == BinLocation.AtStation && b.Station == st) <= 1);

        // lanes ↔ active orders, one ASRS task at a time
        var lanes = await db.Lanes.ToListAsync();
        var active = orders.Where(o => o.Status == OrderStatus.Active).ToList();
        Assert.Equal(active.Count, lanes.Count(l => l.State == LaneState.Active));
        Assert.All(active, o => Assert.Equal(o.Id, lanes.Single(l => l.Id == o.LaneId).OrderId));
        Assert.True(await db.AsrsTasks.CountAsync(t => t.Status == AsrsTaskStatus.Assigned) <= 1);
    }
}
