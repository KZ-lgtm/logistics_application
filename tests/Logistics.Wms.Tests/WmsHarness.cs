using Logistics.Wms.Application;
using Logistics.Wms.Application.Contracts;
using Logistics.Wms.Application.Services;
using Logistics.Wms.Application.Simulation;
using Logistics.Wms.Domain;
using Logistics.Wms.Infrastructure;
using Logistics.Wms.Persistence;
using Microsoft.Data.Sqlite;
using Microsoft.EntityFrameworkCore;
using Microsoft.Extensions.DependencyInjection;

namespace Logistics.Wms.Tests;

public sealed class ManualClock(DateTimeOffset start) : TimeProvider
{
    public DateTimeOffset Now { get; private set; } = start;
    public override DateTimeOffset GetUtcNow() => Now;
    public void Advance(TimeSpan by) => Now += by;
}

/// <summary>WMS services over a private in-memory SQLite database, seeded like production.</summary>
public sealed class WmsHarness : IAsyncDisposable
{
    private readonly SqliteConnection _keepAlive;
    private readonly ServiceProvider _root;
    private AsyncServiceScope _scope;

    private WmsHarness(SqliteConnection keepAlive, ServiceProvider root)
    {
        _keepAlive = keepAlive;
        _root = root;
        _scope = root.CreateAsyncScope();
    }

    public ManualClock Clock => (ManualClock)_root.GetRequiredService<TimeProvider>();
    public WmsDbContext Db => Get<WmsDbContext>();
    public T Get<T>() where T : notnull => _scope.ServiceProvider.GetRequiredService<T>();

    public OrderService Orders => Get<OrderService>();
    public AsrsService Asrs => Get<AsrsService>();
    public PickingService Picking => Get<PickingService>();
    public SortationService Sortation => Get<SortationService>();
    public PutWallService PutWall => Get<PutWallService>();
    public RestockService Restock => Get<RestockService>();
    public ExceptionService Exceptions => Get<ExceptionService>();
    public InventoryService Inventory => Get<InventoryService>();

    /// <param name="inMemory">EF Core in-memory provider (as used by the browser edition) instead of SQLite.</param>
    public static async Task<WmsHarness> CreateAsync(SimulationOptions? simulation = null, bool inMemory = false, bool seed = true)
    {
        var name = $"wms-{Guid.NewGuid():N}";
        var cs = $"Data Source={name};Mode=Memory;Cache=Shared";
        var keepAlive = new SqliteConnection(cs);
        await keepAlive.OpenAsync();
        var services = new ServiceCollection().AddWmsApplication(simulation);
        if (inMemory)
        {
            services.AddDbContext<WmsDbContext>(o => o.UseInMemoryDatabase(name));
            services.AddScoped<Logistics.Wms.Application.Abstractions.IWmsDbContext>(sp => sp.GetRequiredService<WmsDbContext>());
        }
        else
        {
            services.AddWmsInfrastructure(cs);
        }
        services.AddSingleton<TimeProvider>(new ManualClock(new DateTimeOffset(2026, 10, 30, 8, 0, 0, TimeSpan.Zero)));
        var root = services.BuildServiceProvider(new ServiceProviderOptions { ValidateScopes = true });
        if (!seed)
        {
            await using var scope = root.CreateAsyncScope();
            await scope.ServiceProvider.GetRequiredService<WmsDbContext>().Database.EnsureCreatedAsync();
        }
        else if (inMemory) await WmsDatabase.InitializeAsync(root);
        else await root.InitializeWmsDatabaseAsync();
        return new WmsHarness(keepAlive, root);
    }

    /// <summary>Runs simulator rounds, two simulated seconds apart.</summary>
    public async Task RunSimulationAsync(int rounds)
    {
        for (var i = 0; i < rounds; i++)
        {
            await Get<CellSimulator>().StepAsync();
            Clock.Advance(TimeSpan.FromSeconds(2));
            if (i % 100 == 99) await NewScopeAsync();
        }
        await NewScopeAsync();
    }

    /// <summary>Drops tracked entities so the next reads come from the database.</summary>
    public async Task NewScopeAsync()
    {
        await _scope.DisposeAsync();
        _scope = _root.CreateAsyncScope();
    }

    /// <summary>Puts all bins of a SKU to the given quantities (first bin first) through cycle counts.</summary>
    public async Task SetSkuBinsAsync(string skuCode, params int[] quantities)
    {
        var bins = await Inventory.ListBinsAsync(skuCode, null);
        for (var i = 0; i < bins.Count; i++)
            await Inventory.AdjustBinAsync(bins[i].Id, new AdjustBinRequest(i < quantities.Length ? quantities[i] : 0));
    }

    public Task<OrderDto> OrderAsync(string reference, params (string Sku, int Qty)[] lines) =>
        Orders.CreateAsync(new CreateOrderRequest(reference, [.. lines.Select(l => new CreateOrderLineRequest(l.Sku, l.Qty))]));

    /// <summary>R1 fetches the next task and completes it at once; returns the completed task (or null when idle).</summary>
    public async Task<AsrsTaskDto?> RunR1Async()
    {
        var task = await Asrs.RequestNextTaskAsync();
        return task is null ? null : await Asrs.CompleteTaskAsync(task.Id);
    }

    /// <summary>Retrieves a bin, routes it to its stop and lets it arrive; returns the stop.</summary>
    public async Task<StationDto> PresentNextBinAsync()
    {
        var task = await RunR1Async() ?? throw new InvalidOperationException("R1 had nothing to do.");
        Assert.Equal(AsrsTaskKind.Retrieve, task.Kind);
        var route = await Picking.RouteBinAsync(task.BinId);
        Assert.Equal(RouteDecision.Station, route.Decision);
        return await Picking.ArriveAtStationAsync(task.BinId, route.Station!.Value);
    }

    public async ValueTask DisposeAsync()
    {
        await _scope.DisposeAsync();
        await _root.DisposeAsync();
        await _keepAlive.DisposeAsync();
    }
}
