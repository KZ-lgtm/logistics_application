using Logistics.Wms.Application.Contracts;
using Logistics.Wms.Domain;

namespace Logistics.Wms.Tests;

public class OrderTests
{
    [Fact]
    public async Task Invalid_orders_are_rejected()
    {
        await using var h = await WmsHarness.CreateAsync();
        var unknown = await Assert.ThrowsAsync<WmsException>(() => h.OrderAsync("A", ("SKU-9999", 1)));
        Assert.Equal(WmsErrorKind.Validation, unknown.Kind);
        Assert.Equal(WmsErrorKind.Validation, (await Assert.ThrowsAsync<WmsException>(() => h.OrderAsync("B", ("SKU-1001", 0)))).Kind);
        Assert.Equal(WmsErrorKind.Validation, (await Assert.ThrowsAsync<WmsException>(() => h.Orders.CreateAsync(new CreateOrderRequest("C", [])))).Kind);

        await h.OrderAsync("D", ("SKU-1001", 1));
        Assert.Equal(WmsErrorKind.Conflict, (await Assert.ThrowsAsync<WmsException>(() => h.OrderAsync("D", ("SKU-1002", 1)))).Kind);
    }

    [Fact]
    public async Task Lines_of_the_same_sku_are_merged()
    {
        await using var h = await WmsHarness.CreateAsync();
        var o = await h.OrderAsync("M", ("sku-1001", 2), ("SKU-1001", 1), ("SKU-1002", 1));
        Assert.Equal(2, o.Lines.Count);
        Assert.Equal(3, o.Lines.Single(l => l.SkuCode == "SKU-1001").Quantity);
    }

    [Fact]
    public async Task Rolling_waves_fill_the_eight_lanes_by_priority_then_fifo()
    {
        await using var h = await WmsHarness.CreateAsync();
        foreach (var lane in await h.PutWall.ListAsync()) await h.PutWall.SetEnabledAsync(lane.Id, false);

        for (var i = 1; i <= 9; i++) await h.OrderAsync($"O{i}", ("SKU-1001", 1));
        await h.Orders.CreateAsync(new CreateOrderRequest("RUSH", [new("SKU-1002", 1)], Priority: 9));
        foreach (var lane in await h.PutWall.ListAsync()) await h.PutWall.SetEnabledAsync(lane.Id, true);

        var active = await h.Orders.ListAsync(OrderStatus.Active, 0, 100);
        var queued = await h.Orders.ListAsync(OrderStatus.Queued, 0, 100);
        Assert.Equal(8, active.Count);
        Assert.Contains(active, o => o.Reference == "RUSH");
        Assert.Equal(["O8", "O9"], queued.Select(o => o.Reference).Order());
        Assert.Equal(8, active.Select(o => o.LaneId).Distinct().Count());
    }

    [Fact]
    public async Task Active_order_without_picks_can_be_cancelled_and_frees_its_lane()
    {
        await using var h = await WmsHarness.CreateAsync();
        for (var i = 1; i <= 8; i++) await h.OrderAsync($"O{i}", ("SKU-1003", 1));
        var waiting = await h.OrderAsync("O9", ("SKU-1004", 1));
        Assert.Equal(OrderStatus.Queued, waiting.Status);

        var first = (await h.Orders.ListAsync(OrderStatus.Active, 0, 100)).OrderBy(o => o.Id).First();
        var station = await h.PresentNextBinAsync();
        Assert.Contains(station.Picks, p => p.OrderId == first.Id);

        var cancelled = await h.Orders.CancelAsync(first.Id);
        Assert.Equal(OrderStatus.Cancelled, cancelled.Status);
        Assert.Equal(first.LaneId, (await h.Orders.GetAsync(waiting.Id)).LaneId);
        Assert.DoesNotContain((await h.Picking.GetStationAsync(station.Station)).Picks, p => p.OrderId == first.Id);
    }

    [Fact]
    public async Task Order_with_picked_cartons_cannot_be_cancelled()
    {
        await using var h = await WmsHarness.CreateAsync();
        var o = await h.OrderAsync("X", ("SKU-1001", 2));
        var station = await h.PresentNextBinAsync();
        await h.Picking.ConfirmPickAsync(station.Picks[0].PickTaskId);
        var ex = await Assert.ThrowsAsync<WmsException>(() => h.Orders.CancelAsync(o.Id));
        Assert.Equal(WmsErrorKind.Conflict, ex.Kind);
    }
}
