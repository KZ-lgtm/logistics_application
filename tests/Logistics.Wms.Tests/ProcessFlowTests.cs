using Logistics.Wms.Application.Contracts;
using Logistics.Wms.Domain;
using Microsoft.EntityFrameworkCore;

namespace Logistics.Wms.Tests;

/// <summary>The goods-to-robot cycle through the WMS: R1 → picking stop → belt → arch → sorter → put wall → packing → restock.</summary>
public class ProcessFlowTests
{
    [Fact]
    public async Task Order_runs_from_the_shelf_into_its_case_and_the_lane_takes_the_next_order()
    {
        await using var h = await WmsHarness.CreateAsync();
        await h.SetSkuBinsAsync("SKU-1001", 10, 10, 10);
        var order = await h.OrderAsync("ORD-1", ("SKU-1001", 2));
        Assert.Equal(OrderStatus.Active, order.Status);
        Assert.Equal("S1-1", order.LaneId);

        // R1 retrieves a bin of the SKU onto the picking loop
        var retrieve = await h.Asrs.RequestNextTaskAsync();
        Assert.NotNull(retrieve);
        Assert.Equal(AsrsTaskKind.Retrieve, retrieve.Kind);
        Assert.Equal("SKU-1001", retrieve.SkuCode);
        Assert.Equal(retrieve.Id, (await h.Asrs.RequestNextTaskAsync())!.Id);                                            // one task at a time
        await h.Asrs.CompleteTaskAsync(retrieve.Id);
        Assert.Null(await h.Asrs.RequestNextTaskAsync());                                                                // demand covered

        // decision point → P2, arrival → two picks
        var route = await h.Picking.RouteBinAsync(retrieve.BinId);
        Assert.Equal((RouteDecision.Station, PickStation.P2), (route.Decision, route.Station));
        var station = await h.Picking.ArriveAtStationAsync(retrieve.BinId, PickStation.P2);
        Assert.Equal(2, station.Picks.Count);

        var p1 = await h.Picking.ConfirmPickAsync(station.Picks[0].PickTaskId);
        Assert.False(p1.BinReleased);
        var p2 = await h.Picking.ConfirmPickAsync(station.Picks[1].PickTaskId);
        Assert.True(p2.BinReleased);
        Assert.Equal(8, p2.BinQuantityLeft);

        // arch and sorter
        foreach (var pick in station.Picks)
        {
            var scan = await h.Sortation.ScanAsync(new ScanRequest(pick.PickTaskId, "SKU-1001"));
            Assert.Equal((ScanTarget.S1, "S1-1"), (scan.Target, scan.LaneId));
        }
        Assert.False((await h.Sortation.ConfirmSortAsync(new SortConfirmRequest(station.Picks[0].PickTaskId, "S1-1"))).OrderCompleted);
        Assert.True((await h.Sortation.ConfirmSortAsync(new SortConfirmRequest(station.Picks[1].PickTaskId, "S1-1"))).OrderCompleted);
        Assert.Equal(OrderStatus.Completed, (await h.Orders.GetAsync(order.Id)).Status);

        // R1 stores the bin back (dynamic slotting)
        var store = await h.RunR1Async();
        Assert.Equal((AsrsTaskKind.Store, retrieve.BinId), (store!.Kind, store.BinId));
        Assert.NotNull(store.ToSlot);
        Assert.Equal(BinLocation.Rack, (await h.Inventory.GetBinAsync(retrieve.BinId)).Location);

        // packing operator: the next order waits until the lane has an empty case again
        var next = await h.OrderAsync("ORD-2", ("SKU-1002", 1));
        Assert.Equal("S1-2", next.LaneId);
        await h.PutWall.AdvanceAsync("S1-1", LaneEvent.CaseAtOperator);
        await h.PutWall.AdvanceAsync("S1-1", LaneEvent.CaseReplaced);
        var ready = await h.PutWall.AdvanceAsync("S1-1", LaneEvent.EmptyCaseInPosition);
        Assert.Equal(LaneState.Ready, ready.State);
        Assert.Equal(1, ready.CasesCompleted);
        Assert.Equal("S1-1", (await h.OrderAsync("ORD-3", ("SKU-1003", 1))).LaneId);
    }

    [Fact]
    public async Task Second_bin_goes_to_P1_and_a_third_waits_at_the_decision_point()
    {
        await using var h = await WmsHarness.CreateAsync();
        await h.OrderAsync("A", ("SKU-1001", 1));
        await h.OrderAsync("B", ("SKU-1002", 1));
        await h.OrderAsync("C", ("SKU-1003", 1));
        var bins = new List<string>();
        for (var i = 0; i < 3; i++) bins.Add((await h.RunR1Async())!.BinId);

        Assert.Equal(PickStation.P2, (await h.Picking.RouteBinAsync(bins[0])).Station);
        Assert.Equal(PickStation.P1, (await h.Picking.RouteBinAsync(bins[1])).Station);
        Assert.Equal(RouteDecision.Wait, (await h.Picking.RouteBinAsync(bins[2])).Decision);
        var occupied = await Assert.ThrowsAsync<WmsException>(async () =>
        {
            await h.Picking.ArriveAtStationAsync(bins[0], PickStation.P2);
            await h.Picking.ArriveAtStationAsync(bins[2], PickStation.P2);
        });
        Assert.Equal(WmsErrorKind.Conflict, occupied.Kind);
    }

    [Fact]
    public async Task No_read_sends_the_carton_to_the_exception_tote_and_reopens_the_line()
    {
        await using var h = await WmsHarness.CreateAsync();
        await h.SetSkuBinsAsync("SKU-1001", 1, 1, 0);
        var order = await h.OrderAsync("NR", ("SKU-1001", 1));
        var station = await h.PresentNextBinAsync();
        var pick = station.Picks.Single();
        await h.Picking.ConfirmPickAsync(pick.PickTaskId);

        var scan = await h.Sortation.ScanAsync(new ScanRequest(pick.PickTaskId, null));
        Assert.Equal((ScanTarget.Exception, ExceptionReason.NoRead), (scan.Target, scan.ExceptionReason));
        var line = (await h.Orders.GetAsync(order.Id)).Lines.Single();
        Assert.Equal((0, 0), (line.Allocated, line.Picked));

        // the empty bin goes to restock, then R1 fetches another bin for the re-opened line
        Assert.Equal(AsrsTaskKind.ReturnToRestock, (await h.RunR1Async())!.Kind);
        var retry = await h.RunR1Async();
        Assert.Equal((AsrsTaskKind.Retrieve, "SKU-1001"), (retry!.Kind, retry.SkuCode));
        Assert.NotEqual(station.BinId, retry.BinId);

        var open = Assert.Single(await h.Exceptions.ListAsync(openOnly: true, take: 10));
        Assert.Equal(1, (await h.Db.CellState.SingleAsync()).ExceptionToteCount);
        await h.Exceptions.ResolveAsync(open.Id, new ResolveExceptionRequest(ExceptionDisposition.ReturnedToStock, "OP2"));
        Assert.Equal(1, (await h.Inventory.GetStockAsync()).Single(s => s.Code == "SKU-1001").RestockBuffer);
        await Assert.ThrowsAsync<WmsException>(() => h.Exceptions.ResolveAsync(open.Id, new ResolveExceptionRequest(ExceptionDisposition.Scrapped)));
    }

    [Fact]
    public async Task Wrong_bar_code_is_an_exception()
    {
        await using var h = await WmsHarness.CreateAsync();
        await h.OrderAsync("MM", ("SKU-1001", 1));
        var pick = (await h.PresentNextBinAsync()).Picks.Single();
        await h.Picking.ConfirmPickAsync(pick.PickTaskId);
        var scan = await h.Sortation.ScanAsync(new ScanRequest(pick.PickTaskId, "SKU-1002"));
        Assert.Equal(ExceptionReason.BarcodeMismatch, scan.ExceptionReason);
        await Assert.ThrowsAsync<WmsException>(() => h.Sortation.ConfirmSortAsync(new SortConfirmRequest(pick.PickTaskId, "S1-1")));
    }

    [Fact]
    public async Task Sorting_into_the_wrong_lane_is_refused()
    {
        await using var h = await WmsHarness.CreateAsync();
        await h.OrderAsync("WL", ("SKU-1001", 1));
        var pick = (await h.PresentNextBinAsync()).Picks.Single();
        await h.Picking.ConfirmPickAsync(pick.PickTaskId);
        await h.Sortation.ScanAsync(new ScanRequest(pick.PickTaskId, "SKU-1001"));
        var ex = await Assert.ThrowsAsync<WmsException>(() => h.Sortation.ConfirmSortAsync(new SortConfirmRequest(pick.PickTaskId, "S2-3")));
        Assert.Equal(WmsErrorKind.Conflict, ex.Kind);
    }

    [Fact]
    public async Task Short_bin_at_the_stop_releases_the_picks_it_cannot_serve()
    {
        await using var h = await WmsHarness.CreateAsync();
        await h.SetSkuBinsAsync("SKU-1001", 3, 3, 0);
        var order = await h.OrderAsync("SH", ("SKU-1001", 3));
        var station = await h.PresentNextBinAsync();
        Assert.Equal(3, station.Picks.Count);

        // 3D image shows only one carton
        var after = await h.Picking.ReportBinCountAsync(station.BinId!, new BinCountRequest(1, "vision"));
        Assert.Single(after.Picks);
        Assert.Equal(1, (await h.Orders.GetAsync(order.Id)).Lines.Single().Allocated);
        var adj = (await h.Inventory.ListTransactionsAsync(null, station.BinId, 10)).First();
        Assert.Equal((InventoryTransactionType.Adjustment, -2), (adj.Type, adj.Quantity));

        await h.Picking.ConfirmPickAsync(after.Picks[0].PickTaskId);
        Assert.Equal(AsrsTaskKind.ReturnToRestock, (await h.RunR1Async())!.Kind);       // empty bin out
        var retry = await h.RunR1Async();                                               // the other bin for the 2 missing
        Assert.Equal(AsrsTaskKind.Retrieve, retry!.Kind);
        Assert.NotEqual(station.BinId, retry.BinId);
    }

    [Fact]
    public async Task Empty_bin_is_refilled_at_restock_and_stored_again()
    {
        await using var h = await WmsHarness.CreateAsync();
        await h.SetSkuBinsAsync("SKU-1001", 2, 0, 0);
        await h.OrderAsync("E", ("SKU-1001", 2));
        var station = await h.PresentNextBinAsync();
        foreach (var p in station.Picks) await h.Picking.ConfirmPickAsync(p.PickTaskId);

        var toRestock = await h.RunR1Async();
        Assert.Equal((AsrsTaskKind.ReturnToRestock, station.BinId), (toRestock!.Kind, toRestock.BinId));
        var waiting = Assert.Single(await h.Restock.ListAsync());
        Assert.Equal(32, waiting.SuggestedAdd);

        var refilled = await h.Restock.CompleteAsync(station.BinId!, new RestockRequest(User: "OP1"));
        Assert.Equal((32, BinLocation.RestockReturn), (refilled.Quantity, refilled.Location));
        var store = await h.RunR1Async();
        Assert.Equal((AsrsTaskKind.StoreRestocked, station.BinId), (store!.Kind, store.BinId));

        // the two empty bins left in the rack follow, at most three on the restock loop
        Assert.Equal(AsrsTaskKind.RackToRestock, (await h.RunR1Async())!.Kind);
        Assert.Equal(AsrsTaskKind.RackToRestock, (await h.RunR1Async())!.Kind);
        Assert.Null(await h.RunR1Async());
    }

    [Fact]
    public async Task Empty_bin_can_take_a_new_sku_and_uses_the_exception_buffer()
    {
        await using var h = await WmsHarness.CreateAsync();
        await h.SetSkuBinsAsync("SKU-1001", 0, 5, 5);
        var task = await h.RunR1Async();
        Assert.Equal(AsrsTaskKind.RackToRestock, task!.Kind);

        var sku = await h.Db.Skus.SingleAsync(s => s.Code == "SKU-1005");
        sku.RestockBuffer = 3;
        await h.Db.SaveChangesAsync();
        var bin = await h.Restock.CompleteAsync(task.BinId, new RestockRequest(Quantity: 6, SkuCode: "SKU-1005"));
        Assert.Equal(("SKU-1005", 6), (bin.SkuCode, bin.Quantity));
        Assert.Equal(0, (await h.Db.Skus.SingleAsync(s => s.Code == "SKU-1005")).RestockBuffer);

        var tx = await h.Inventory.ListTransactionsAsync("SKU-1005", null, 10);
        Assert.Equal(3, tx.Where(t => t.Type == InventoryTransactionType.Restock).Sum(t => t.Quantity));   // +6 into the bin, −3 out of the buffer
    }

    [Fact]
    public async Task Moving_bins_cannot_be_counted_and_blocked_slots_are_not_used()
    {
        await using var h = await WmsHarness.CreateAsync();
        await h.OrderAsync("C", ("SKU-1001", 1));
        var retrieve = await h.RunR1Async();
        await Assert.ThrowsAsync<WmsException>(() => h.Inventory.AdjustBinAsync(retrieve!.BinId, new AdjustBinRequest(3)));

        foreach (var slot in await h.Inventory.ListSlotsAsync())
            if (slot.BinId is null && !slot.Reserved) await h.Inventory.SetSlotBlockedAsync(slot.Id, true);
        // every free slot (incl. the one just emptied) is blocked: the bin coming back has nowhere to go
        await h.Picking.RouteBinAsync(retrieve!.BinId);
        var station = await h.Picking.ArriveAtStationAsync(retrieve.BinId, PickStation.P2);
        await h.Picking.ConfirmPickAsync(station.Picks.Single().PickTaskId);
        Assert.Null(await h.Asrs.RequestNextTaskAsync());
        await h.Inventory.SetSlotBlockedAsync((await h.Inventory.ListSlotsAsync()).First(s => s.Blocked).Id, false);
        Assert.Equal(AsrsTaskKind.Store, (await h.Asrs.RequestNextTaskAsync())!.Kind);
    }
}
