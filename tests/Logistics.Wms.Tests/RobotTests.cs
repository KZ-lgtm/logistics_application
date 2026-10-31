using Logistics.Wms.Application.Contracts;
using Logistics.Wms.Application.Services;
using Logistics.Wms.Application.Simulation;
using Logistics.Wms.Domain;
using Microsoft.EntityFrameworkCore;

namespace Logistics.Wms.Tests;

public class RobotTests
{
    private static RobotService Robots(WmsHarness h) => h.Get<RobotService>();

    [Fact]
    public async Task Five_robots_are_registered_with_their_roles()
    {
        await using var h = await WmsHarness.CreateAsync();
        var robots = await Robots(h).ListAsync();
        Assert.Equal(["R1", "R2", "R3", "S1", "S2"], robots.Select(r => r.Id).Order());
        Assert.Equal(PickStation.P1, (await h.Db.Robots.SingleAsync(r => r.Id == "R2")).Station);
        Assert.Equal(PickStation.P2, (await h.Db.Robots.SingleAsync(r => r.Id == "R3")).Station);
        Assert.All(robots, r => Assert.True(r.Available));
    }

    [Fact]
    public async Task R1_gets_no_task_while_faulted_or_not_in_auto()
    {
        await using var h = await WmsHarness.CreateAsync();
        await h.OrderAsync("A", ("SKU-1001", 1));
        await Robots(h).ReportFaultAsync("R1", new RobotFaultRequest("50056", "joint collision"));
        Assert.Null(await h.Asrs.RequestNextTaskAsync());
        await Robots(h).ClearFaultAsync("R1");
        await Robots(h).SetModeAsync("r1", new SetRobotModeRequest(RobotMode.Manual));
        Assert.Null(await Robots(h).NextTaskAsync("R1"));
        await Robots(h).SetModeAsync("R1", new SetRobotModeRequest(RobotMode.Auto));
        var t = await Robots(h).NextTaskAsync("R1");
        Assert.Equal("Retrieve", t!.Kind);
        var r1 = await Robots(h).GetAsync("R1");
        Assert.Equal((RobotState.Working, 1), (r1.State, r1.Faults));
        Assert.Equal(4, (await Robots(h).ListEventsAsync("R1", 10)).Count);   // fault, cleared, → manual, → auto
    }

    [Fact]
    public async Task Picker_fault_hands_its_bin_back_and_the_other_stop_takes_over()
    {
        await using var h = await WmsHarness.CreateAsync();
        await h.SetSkuBinsAsync("SKU-1001", 5, 5, 0);
        var order = await h.OrderAsync("PF", ("SKU-1001", 3));
        var station = await h.PresentNextBinAsync();
        Assert.Equal(PickStation.P2, station.Station);              // R3's stop
        await h.Picking.ConfirmPickAsync(station.Picks[0].PickTaskId);

        await Robots(h).ReportFaultAsync("R3", new RobotFaultRequest("VAC-01", "vacuum lost"));
        var bin = await h.Inventory.GetBinAsync(station.BinId!);
        Assert.Equal(BinLocation.PickReturn, bin.Location);
        Assert.Equal((1, 1), ((await h.Orders.GetAsync(order.Id)).Lines.Single().Allocated, (await h.Orders.GetAsync(order.Id)).Lines.Single().Picked));
        Assert.Contains(await Robots(h).ListEventsAsync("R3", 10), e => e.Kind == RobotEventKind.WorkReassigned);

        // R1 stores the released bin, then fetches again; the decision point now sends it to P1 (R2)
        Assert.Equal(AsrsTaskKind.Store, (await h.RunR1Async())!.Kind);
        var retry = await h.RunR1Async();
        Assert.Equal(AsrsTaskKind.Retrieve, retry!.Kind);
        Assert.Equal(PickStation.P1, (await h.Picking.RouteBinAsync(retry.BinId)).Station);
        Assert.Equal(2, (await h.Picking.ArriveAtStationAsync(retry.BinId, PickStation.P1)).Picks.Count);
    }

    [Fact]
    public async Task Sorter_fault_holds_its_lanes_and_orders_until_it_is_back()
    {
        await using var h = await WmsHarness.CreateAsync();
        var s1Order = await h.OrderAsync("ON-S1", ("SKU-1001", 1));
        Assert.Equal("S1-1", s1Order.LaneId);
        await Robots(h).ReportFaultAsync("S1", new RobotFaultRequest("E-417"));

        // new orders go to S2's lanes only
        for (var i = 0; i < 6; i++) await h.OrderAsync($"N{i}", ("SKU-1002", 1));
        var active = await h.Orders.ListAsync(OrderStatus.Active, 0, 50);
        Assert.All(active.Where(o => o.Id != s1Order.Id), o => Assert.StartsWith("S2-", o.LaneId));
        Assert.Equal(2, (await h.Orders.ListAsync(OrderStatus.Queued, 0, 50)).Count);

        // the S1 order gets no bin; a carton already picked for it goes to the exception tote
        Assert.Equal(0, await h.Get<DemandService>().OpenQuantityAsync((await h.Db.Skus.SingleAsync(s => s.Code == "SKU-1001")).Id));
        await Robots(h).ClearFaultAsync("S1");
        var station = await h.PresentNextBinAsync();
        var pick = station.Picks.Single(p => p.OrderId == s1Order.Id);
        await h.Picking.ConfirmPickAsync(pick.PickTaskId);
        await Robots(h).ReportFaultAsync("S1", new RobotFaultRequest("E-417"));
        Assert.Equal(ExceptionReason.SorterUnavailable, (await h.Sortation.ScanAsync(new ScanRequest(pick.PickTaskId, "SKU-1001"))).ExceptionReason);

        // back in service: the queued orders take S1's free lanes
        await Robots(h).ClearFaultAsync("S1");
        Assert.Empty(await h.Orders.ListAsync(OrderStatus.Queued, 0, 50));
    }

    [Fact]
    public async Task Robot_dispatch_runs_a_carton_through_all_five_robots_and_counts_cycles()
    {
        await using var h = await WmsHarness.CreateAsync();
        var robots = Robots(h);
        await h.OrderAsync("D", ("SKU-1006", 1));

        var r1 = await robots.NextTaskAsync("R1");
        h.Clock.Advance(TimeSpan.FromSeconds(9));
        await robots.CompleteTaskAsync("R1", r1!.Ref);
        var route = await h.Picking.RouteBinAsync(r1.BinId!);
        await h.Picking.ArriveAtStationAsync(r1.BinId!, route.Station!.Value);

        var picker = CellLayout.PickerOf(route.Station!.Value);
        Assert.Null(await robots.NextTaskAsync(picker == "R2" ? "R3" : "R2"));
        var pick = await robots.NextTaskAsync(picker);
        Assert.Equal("Pick", pick!.Kind);
        h.Clock.Advance(TimeSpan.FromSeconds(3));
        await robots.CompleteTaskAsync(picker, pick.Ref);
        await h.Sortation.ScanAsync(new ScanRequest(long.Parse(pick.Ref), "SKU-1006"));

        var sort = await robots.NextTaskAsync("S1");
        Assert.Equal(("Sort", "S1-1"), (sort!.Kind, sort.LaneId));
        h.Clock.Advance(TimeSpan.FromSeconds(2));
        var done = await robots.CompleteTaskAsync("S1", sort.Ref);
        Assert.Contains("complete", done.Description);
        await Assert.ThrowsAsync<WmsException>(() => robots.CompleteTaskAsync("S2", sort.Ref));

        var all = (await robots.ListAsync()).ToDictionary(r => r.Id);
        Assert.Equal((1, 9.0), (all["R1"].Cycles, all["R1"].LastCycleSeconds));
        Assert.Equal((1, 3.0), (all[picker].Cycles, all[picker].LastCycleSeconds));
        Assert.Equal((1, 2.0), (all["S1"].Cycles, all["S1"].LastCycleSeconds));
        Assert.All(all.Values, r => Assert.Equal(RobotState.Idle, r.State));
        Assert.True(all["R1"].Utilization > 0.5);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task Robot_cycles_match_the_work_done(bool inMemory)
    {
        await using var h = await WmsHarness.CreateAsync(new SimulationOptions { Seed = 9 }, inMemory);
        await h.RunSimulationAsync(300);
        var robots = (await Robots(h).ListAsync()).ToDictionary(r => r.Id);
        var db = h.Db;
        Assert.Equal(await db.AsrsTasks.CountAsync(t => t.Status == AsrsTaskStatus.Completed), robots["R1"].Cycles);
        foreach (var st in Enum.GetValues<PickStation>())
            Assert.Equal(await db.PickTasks.CountAsync(p => p.Station == st && p.PickedAt != null), robots[CellLayout.PickerOf(st)].Cycles);
        foreach (var so in Enum.GetValues<Sorter>())
            Assert.Equal(await db.PickTasks.CountAsync(p => p.TargetSorter == so && p.Status == PickStatus.Sorted), robots[CellLayout.SorterRobotOf(so)].Cycles);
    }

    [Fact]
    public async Task With_head_sensors_R1_only_stores_the_bin_that_is_physically_at_the_head()
    {
        await using var h = await WmsHarness.CreateAsync();
        await h.OrderAsync("A", ("SKU-1001", 1));
        await h.OrderAsync("B", ("SKU-1002", 1));
        var first = await h.PresentNextBinAsync();
        var second = await h.PresentNextBinAsync();
        foreach (var st in new[] { first, second }) await h.Picking.ConfirmPickAsync(st.Picks.Single().PickTaskId);

        // both bins are booked back; the cell sees only the second one at the return head and the out heads busy
        var t = await h.Asrs.RequestNextTaskAsync(new AsrsHeads(second.BinId, null, PickOutFree: false, RestockOutFree: false));
        Assert.Equal((AsrsTaskKind.Store, second.BinId), (t!.Kind, t.BinId));
        await h.Asrs.CompleteTaskAsync(t.Id);
        Assert.Null(await h.Asrs.RequestNextTaskAsync(new AsrsHeads(null, null, PickOutFree: false, RestockOutFree: false)));
        Assert.Equal(first.BinId, (await h.Asrs.RequestNextTaskAsync(new AsrsHeads(first.BinId, null)))!.BinId);
    }

    [Fact]
    public async Task Long_run_with_robot_faults_stays_consistent()
    {
        await using var h = await WmsHarness.CreateAsync(new SimulationOptions { NoReadRate = 0.01, Seed = 21 }, inMemory: true);
        var rng = new Random(4);
        string[] ids = ["R1", "R2", "R3", "S1", "S2"];
        for (var block = 0; block < 12; block++)
        {
            var victim = ids[rng.Next(ids.Length)];
            await h.Get<RobotService>().ReportFaultAsync(victim, new RobotFaultRequest("SIM", "injected"));
            await h.RunSimulationAsync(40);
            await h.Get<RobotService>().ClearFaultAsync(victim);
            await h.RunSimulationAsync(80);
        }
        var robots = await h.Get<RobotService>().ListAsync();
        Assert.All(robots, r => Assert.True(r.Cycles > 0, $"{r.Id} never worked"));
        Assert.All(robots, r => Assert.True(r.Available));
        Assert.Equal(12, robots.Sum(r => r.Faults));
        Assert.True(await h.Db.Orders.CountAsync(o => o.Status == OrderStatus.Completed) > 200);
        await SimulationTests.AssertConsistentAsync(h.Db, ledgerHasAllPicks: true);
    }
}
