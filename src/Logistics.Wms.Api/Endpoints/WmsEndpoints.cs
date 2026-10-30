using Logistics.Wms.Application.Contracts;
using Logistics.Wms.Application.Services;
using Logistics.Wms.Application.Simulation;
using Logistics.Wms.Domain;

namespace Logistics.Wms.Api.Endpoints;

/// <summary>
/// HTTP surface of the WMS. Host-facing: orders, inventory, reports. Cell-facing (WCS / PLC / operator panels): ASRS tasks for R1,
/// picking stops, bar-code arch and sorters, put-wall lanes, restock station and exception tote.
/// </summary>
public static class WmsEndpoints
{
    public static void MapOrderEndpoints(this RouteGroupBuilder api)
    {
        var g = api.MapGroup("/orders").WithTags("Orders");
        g.MapGet("/", (OrderService s, OrderStatus? status, int? skip, int? take, CancellationToken ct) => s.ListAsync(status, skip ?? 0, take ?? 100, ct));
        g.MapGet("/{id:int}", (OrderService s, int id, CancellationToken ct) => s.GetAsync(id, ct));
        g.MapPost("/", async (OrderService s, CreateOrderRequest request, CancellationToken ct) =>
        {
            var order = await s.CreateAsync(request, ct);
            return Results.Created($"/api/orders/{order.Id}", order);
        });
        g.MapPost("/{id:int}/cancel", (OrderService s, int id, CancellationToken ct) => s.CancelAsync(id, ct));
    }

    public static void MapAsrsEndpoints(this RouteGroupBuilder api)
    {
        var g = api.MapGroup("/asrs/tasks").WithTags("ASRS (R1)");
        g.MapGet("/", (AsrsService s, int? take, CancellationToken ct) => s.ListAsync(take ?? 50, ct));
        g.MapGet("/current", async (AsrsService s, CancellationToken ct) => await s.GetCurrentTaskAsync(ct) is { } t ? Results.Ok(t) : Results.NoContent());
        g.MapPost("/next", async (AsrsService s, CancellationToken ct) => await s.RequestNextTaskAsync(ct) is { } t ? Results.Ok(t) : Results.NoContent());
        g.MapPost("/{id:int}/complete", (AsrsService s, int id, CancellationToken ct) => s.CompleteTaskAsync(id, ct));
        g.MapPost("/{id:int}/cancel", (AsrsService s, int id, CancellationToken ct) => s.CancelTaskAsync(id, ct));
    }

    public static void MapPickingEndpoints(this RouteGroupBuilder api)
    {
        var g = api.MapGroup("/picking").WithTags("Picking (R2 / R3)");
        g.MapPost("/bins/{binId}/route", (PickingService s, string binId, CancellationToken ct) => s.RouteBinAsync(binId, ct));
        g.MapGet("/stations/{station}", (PickingService s, PickStation station, CancellationToken ct) => s.GetStationAsync(station, ct));
        g.MapPost("/stations/{station}/arrive/{binId}", (PickingService s, PickStation station, string binId, CancellationToken ct) => s.ArriveAtStationAsync(binId, station, ct));
        g.MapPost("/picks/{id:long}/confirm", (PickingService s, long id, CancellationToken ct) => s.ConfirmPickAsync(id, ct));
        g.MapPost("/bins/{binId}/count", (PickingService s, string binId, BinCountRequest request, CancellationToken ct) => s.ReportBinCountAsync(binId, request, ct));
    }

    public static void MapSortationEndpoints(this RouteGroupBuilder api)
    {
        var g = api.MapGroup("/sortation").WithTags("Sortation (arch, S1 / S2)");
        g.MapGet("/belt", (SortationService s, CancellationToken ct) => s.ListOnBeltAsync(ct));
        g.MapPost("/scan", (SortationService s, ScanRequest request, CancellationToken ct) => s.ScanAsync(request, ct));
        g.MapPost("/sort", (SortationService s, SortConfirmRequest request, CancellationToken ct) => s.ConfirmSortAsync(request, ct));
        g.MapPost("/items/{id:long}/missed", (SortationService s, long id, CancellationToken ct) => s.ReportMissedAsync(id, ct));
    }

    public static void MapPutWallEndpoints(this RouteGroupBuilder api)
    {
        var g = api.MapGroup("/putwall/lanes").WithTags("Put wall");
        g.MapGet("/", (PutWallService s, CancellationToken ct) => s.ListAsync(ct));
        g.MapPost("/{id}/events/{laneEvent}", (PutWallService s, string id, LaneEvent laneEvent, CancellationToken ct) => s.AdvanceAsync(id, laneEvent, ct));
        g.MapPost("/{id}/enable", (PutWallService s, string id, CancellationToken ct) => s.SetEnabledAsync(id, true, ct));
        g.MapPost("/{id}/disable", (PutWallService s, string id, CancellationToken ct) => s.SetEnabledAsync(id, false, ct));
    }

    public static void MapRestockEndpoints(this RouteGroupBuilder api)
    {
        var g = api.MapGroup("/restock").WithTags("Restock");
        g.MapGet("/", (RestockService s, CancellationToken ct) => s.ListAsync(ct));
        g.MapPost("/{binId}/complete", (RestockService s, string binId, RestockRequest? request, CancellationToken ct) =>
            s.CompleteAsync(binId, request ?? new RestockRequest(), ct));
    }

    public static void MapInventoryEndpoints(this RouteGroupBuilder api)
    {
        var g = api.MapGroup("/inventory").WithTags("Inventory");
        g.MapGet("/skus", (InventoryService s, CancellationToken ct) => s.GetStockAsync(ct));
        g.MapPost("/skus", async (InventoryService s, CreateSkuRequest request, CancellationToken ct) =>
        {
            var sku = await s.CreateSkuAsync(request, ct);
            return Results.Created($"/api/inventory/skus/{sku.Code}", sku);
        });
        g.MapGet("/bins", (InventoryService s, string? sku, BinLocation? location, CancellationToken ct) => s.ListBinsAsync(sku, location, ct));
        g.MapGet("/bins/{id}", (InventoryService s, string id, CancellationToken ct) => s.GetBinAsync(id, ct));
        g.MapPost("/bins/{id}/adjust", (InventoryService s, string id, AdjustBinRequest request, CancellationToken ct) => s.AdjustBinAsync(id, request, ct));
        g.MapGet("/slots", (InventoryService s, CancellationToken ct) => s.ListSlotsAsync(ct));
        g.MapPost("/slots/{id:int}/block", (InventoryService s, int id, CancellationToken ct) => s.SetSlotBlockedAsync(id, true, ct));
        g.MapPost("/slots/{id:int}/unblock", (InventoryService s, int id, CancellationToken ct) => s.SetSlotBlockedAsync(id, false, ct));
        g.MapGet("/transactions", (InventoryService s, string? sku, string? bin, int? take, CancellationToken ct) => s.ListTransactionsAsync(sku, bin, take ?? 200, ct));
    }

    public static void MapExceptionEndpoints(this RouteGroupBuilder api)
    {
        var g = api.MapGroup("/exceptions").WithTags("Exceptions");
        g.MapGet("/", (ExceptionService s, bool? openOnly, int? take, CancellationToken ct) => s.ListAsync(openOnly ?? true, take ?? 100, ct));
        g.MapPost("/{id:int}/resolve", (ExceptionService s, int id, ResolveExceptionRequest request, CancellationToken ct) => s.ResolveAsync(id, request, ct));
        g.MapPost("/tote/empty", async (ExceptionService s, CancellationToken ct) => Results.Ok(new { emptied = await s.EmptyToteAsync(ct) }));
    }

    public static void MapDashboardEndpoints(this RouteGroupBuilder api) =>
        api.MapGet("/dashboard", (DashboardService s, CancellationToken ct) => s.GetAsync(ct)).WithTags("Dashboard");

    public static void MapSimulationEndpoints(this RouteGroupBuilder api)
    {
        var g = api.MapGroup("/simulation").WithTags("Simulation");
        g.MapGet("/", (SimulationState s) => Status(s));
        g.MapPost("/start", (SimulationState s) => { s.Running = true; return Status(s); });
        g.MapPost("/stop", (SimulationState s) => { s.Running = false; return Status(s); });
        g.MapPost("/step", async (CellSimulator sim, SimulationState s, int? rounds, CancellationToken ct) =>
        {
            for (var i = 0; i < Math.Clamp(rounds ?? 1, 1, 1000); i++) await sim.StepAsync(ct);
            return Status(s);
        });
    }

    private static object Status(SimulationState s) => new { s.Running, s.Rounds, s.OrdersGenerated, s.Options };
}
