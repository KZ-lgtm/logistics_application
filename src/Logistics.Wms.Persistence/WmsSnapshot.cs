using System.Text.Json;
using System.Text.Json.Serialization;
using System.Text.Json.Serialization.Metadata;
using Logistics.Wms.Application.Services;
using Logistics.Wms.Domain;
using Microsoft.EntityFrameworkCore;

namespace Logistics.Wms.Persistence;

/// <summary>The complete WMS state as plain rows (no navigations), for backups and for the browser edition's local storage.</summary>
public sealed class WmsSnapshot
{
    public const int CurrentVersion = 1;

    public int Version { get; set; } = CurrentVersion;
    public DateTime SavedAt { get; set; }
    public List<Sku> Skus { get; set; } = [];
    public List<StorageSlot> Slots { get; set; } = [];
    public List<Bin> Bins { get; set; } = [];
    public List<Order> Orders { get; set; } = [];
    public List<OrderLine> OrderLines { get; set; } = [];
    public List<PutWallLane> Lanes { get; set; } = [];
    public List<AsrsTask> AsrsTasks { get; set; } = [];
    public List<PickTask> PickTasks { get; set; } = [];
    public List<ExceptionItem> ExceptionItems { get; set; } = [];
    public List<InventoryTransaction> InventoryTransactions { get; set; } = [];
    public List<CellState> CellState { get; set; } = [];
    public List<Robot> Robots { get; set; } = [];
    public List<RobotEvent> RobotEvents { get; set; } = [];

    /// <summary>Rows only: computed (get-only) properties and navigations are left out, enums are written as names.</summary>
    public static readonly JsonSerializerOptions JsonOptions = new(JsonSerializerDefaults.Web)
    {
        Converters = { new JsonStringEnumConverter() },
        DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
        TypeInfoResolver = new DefaultJsonTypeInfoResolver { Modifiers = { RowsOnly } },
    };

    private static readonly HashSet<Type> Entities =
        [typeof(Sku), typeof(StorageSlot), typeof(Bin), typeof(Order), typeof(OrderLine), typeof(PutWallLane), typeof(AsrsTask),
         typeof(PickTask), typeof(ExceptionItem), typeof(InventoryTransaction), typeof(CellState), typeof(Robot), typeof(RobotEvent)];

    private static void RowsOnly(JsonTypeInfo info)
    {
        if (!Entities.Contains(info.Type)) return;
        foreach (var p in info.Properties.ToList())
        {
            var navigation = Entities.Contains(p.PropertyType) || p.PropertyType == typeof(List<OrderLine>);
            if (p.Set is null || navigation) info.Properties.Remove(p);
        }
    }

    public static async Task<WmsSnapshot> CaptureAsync(WmsDbContext db, DateTime savedAt, CancellationToken ct = default) => new()
    {
        SavedAt = savedAt,
        Skus = await db.Skus.AsNoTracking().OrderBy(x => x.Id).ToListAsync(ct),
        Slots = await db.Slots.AsNoTracking().OrderBy(x => x.Id).ToListAsync(ct),
        Bins = await db.Bins.AsNoTracking().OrderBy(x => x.Id).ToListAsync(ct),
        Orders = await db.Orders.AsNoTracking().OrderBy(x => x.Id).ToListAsync(ct),
        OrderLines = await db.OrderLines.AsNoTracking().OrderBy(x => x.Id).ToListAsync(ct),
        Lanes = await db.Lanes.AsNoTracking().OrderBy(x => x.Id).ToListAsync(ct),
        AsrsTasks = await db.AsrsTasks.AsNoTracking().OrderBy(x => x.Id).ToListAsync(ct),
        PickTasks = await db.PickTasks.AsNoTracking().OrderBy(x => x.Id).ToListAsync(ct),
        ExceptionItems = await db.ExceptionItems.AsNoTracking().OrderBy(x => x.Id).ToListAsync(ct),
        InventoryTransactions = await db.InventoryTransactions.AsNoTracking().OrderBy(x => x.Id).ToListAsync(ct),
        CellState = await db.CellState.AsNoTracking().ToListAsync(ct),
        Robots = await db.Robots.AsNoTracking().OrderBy(x => x.Id).ToListAsync(ct),
        RobotEvents = await db.RobotEvents.AsNoTracking().OrderBy(x => x.Id).ToListAsync(ct),
    };

    public string ToJson() => JsonSerializer.Serialize(this, JsonOptions);

    public static WmsSnapshot FromJson(string json)
    {
        var snapshot = JsonSerializer.Deserialize<WmsSnapshot>(json, JsonOptions) ?? throw WmsException.Invalid("Empty snapshot.");
        if (snapshot.Version != CurrentVersion) throw WmsException.Invalid($"Snapshot version {snapshot.Version} is not supported (expected {CurrentVersion}).");
        if (snapshot.Skus.Count == 0 || snapshot.Slots.Count == 0 || snapshot.CellState.Count != 1) throw WmsException.Invalid("Snapshot is incomplete.");
        return snapshot;
    }

    /// <summary>Replaces the whole database content with this snapshot.</summary>
    public async Task RestoreAsync(WmsDbContext db, CancellationToken ct = default)
    {
        await db.Database.EnsureDeletedAsync(ct);
        await db.Database.EnsureCreatedAsync(ct);
        db.ChangeTracker.Clear();
        db.Skus.AddRange(Skus);
        db.Slots.AddRange(Slots);
        db.Lanes.AddRange(Lanes);
        db.CellState.AddRange(CellState);
        db.Bins.AddRange(Bins);
        db.Orders.AddRange(Orders);
        db.OrderLines.AddRange(OrderLines);
        db.AsrsTasks.AddRange(AsrsTasks);
        db.PickTasks.AddRange(PickTasks);
        db.ExceptionItems.AddRange(ExceptionItems);
        db.InventoryTransactions.AddRange(InventoryTransactions);
        db.Robots.AddRange(Robots);
        db.RobotEvents.AddRange(RobotEvents);
        await db.SaveChangesAsync(ct);
        await WmsSeeder.EnsureRobotsAsync(db, SavedAt, ct);    // snapshots from before the robot registry
        db.ChangeTracker.Clear();
    }
}
