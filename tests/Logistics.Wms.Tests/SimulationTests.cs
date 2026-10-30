using Logistics.Wms.Application.Simulation;
using Logistics.Wms.Domain;
using Microsoft.EntityFrameworkCore;

namespace Logistics.Wms.Tests;

/// <summary>Long runs of the cell simulator against the real services; checks the invariants the 3D simulation verifies.</summary>
public class SimulationTests
{
    [Theory]
    [InlineData(true, 0.005)]
    [InlineData(false, 0.05)]
    public async Task Long_run_keeps_stock_bins_lanes_and_orders_consistent(bool pareto, double noRead)
    {
        await using var h = await WmsHarness.CreateAsync(new SimulationOptions { ParetoDemand = pareto, NoReadRate = noRead, Seed = 11 });
        for (var i = 0; i < 1500; i++)
        {
            await h.Get<CellSimulator>().StepAsync();
            h.Clock.Advance(TimeSpan.FromSeconds(2));
            if (i % 100 == 0) await h.NewScopeAsync();
        }
        await h.NewScopeAsync();
        var db = h.Db;

        var orders = await db.Orders.Include(o => o.Lines).ToListAsync();
        Assert.True(orders.Count(o => o.Status == OrderStatus.Completed) > 300, "throughput");
        Assert.All(orders.Where(o => o.Status == OrderStatus.Completed), o => Assert.All(o.Lines, l => Assert.Equal(l.Quantity, l.Sorted)));
        Assert.All(orders.SelectMany(o => o.Lines), l =>
        {
            Assert.InRange(l.Sorted, 0, l.Quantity);
            Assert.InRange(l.Picked, l.Sorted, l.Allocated);
            Assert.InRange(l.Allocated, 0, l.Quantity);
        });

        // stock ledger = bins + exception buffer
        var bins = await db.Bins.Include(b => b.Sku).ToListAsync();
        var skus = await db.Skus.ToListAsync();
        foreach (var sku in skus)
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
        Assert.Equal(-picked, await db.InventoryTransactions.Where(t => t.Type == InventoryTransactionType.Pick).SumAsync(t => t.Quantity));
        if (noRead > 0.01) Assert.True(await db.ExceptionItems.AnyAsync(e => e.Reason == ExceptionReason.NoRead));

        // slots ↔ bins
        var slots = await db.Slots.ToListAsync();
        Assert.All(bins.Where(b => b.Location == BinLocation.Rack), b => Assert.Equal(b.Id, slots.Single(s => s.Id == b.SlotId).BinId));
        Assert.All(bins.Where(b => b.Location != BinLocation.Rack), b => Assert.Null(b.SlotId));
        Assert.Equal(slots.Count(s => s.BinId != null), bins.Count(b => b.Location == BinLocation.Rack));
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
