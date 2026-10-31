using Logistics.Wms.Application.Abstractions;
using Logistics.Wms.Domain;
using Microsoft.EntityFrameworkCore;

namespace Logistics.Wms.Application.Services;

/// <summary>
/// Initial master data matching the 3D simulation: 40 SKUs (sizes S / M / L), the 144 slots of the cylinder shelf, the 8 put-wall
/// lanes and 3 bins per SKU filled to 25…100 % of capacity, spread over the shelf.
/// </summary>
public static class WmsSeeder
{
    public const int SkuCount = 40;
    public const int BinsPerSku = 3;

    public static async Task SeedAsync(IWmsDbContext db, TimeProvider clock, int seed = 42, CancellationToken ct = default)
    {
        if (await db.Skus.AnyAsync(ct)) return;
        var now = clock.GetUtcNow().UtcDateTime;
        var rng = new Random(seed);

        var skus = Enumerable.Range(0, SkuCount).Select(i =>
        {
            var size = i % 5 == 4 ? SkuSize.L : i % 3 == 2 ? SkuSize.M : SkuSize.S;
            return new Sku
            {
                Code = $"SKU-{1001 + i}", Name = $"{size switch { SkuSize.S => "Small", SkuSize.M => "Medium", _ => "Large" }} carton {1001 + i}",
                Size = size, WeightKg = size switch { SkuSize.S => 0.35, SkuSize.M => 0.7, _ => 1.6 },
                ReorderPoint = CellLayout.BinCapacity(size) / 2,
            };
        }).ToList();
        db.Skus.AddRange(skus);

        var slots = new List<StorageSlot>();
        for (var c = 0; c < CellLayout.Columns; c++)
        {
            if (CellLayout.GapColumns.Contains(c)) continue;
            for (var l = 0; l < CellLayout.Levels; l++)
                slots.Add(new StorageSlot { Column = c, Level = l, Angle = CellLayout.ColumnAngle(c), Height = CellLayout.LevelHeight(l) });
        }
        db.Slots.AddRange(slots);

        db.Lanes.AddRange(CellLayout.Lanes.Select(x => new PutWallLane
        {
            Id = CellLayout.LaneId(x.Sorter, x.Index), Sorter = x.Sorter, Index = x.Index, State = LaneState.Ready, StateSince = now,
        }));
        db.CellState.Add(new CellState());
        await db.SaveChangesAsync(ct);
        await EnsureRobotsAsync(db, now, ct);

        // bins into a shuffled set of slots (SKUs mixed around the ring)
        var order = slots.OrderBy(_ => rng.Next()).ToList();
        var n = 0;
        foreach (var sku in skus)
        {
            for (var k = 0; k < BinsPerSku; k++)
            {
                var slot = order[n++];
                var qty = (int)Math.Round(sku.BinCapacity * (0.25 + 0.75 * rng.NextDouble()));
                var bin = new Bin { Id = $"B{n:000}", Sku = sku, Quantity = qty, Location = BinLocation.Rack, Slot = slot, LastMovedAt = now };
                slot.BinId = bin.Id;
                db.Bins.Add(bin);
                db.InventoryTransactions.Add(new InventoryTransaction
                {
                    At = now, Type = InventoryTransactionType.InitialStock, SkuId = sku.Id, BinId = bin.Id, Quantity = qty, BinQuantityAfter = qty,
                    Reference = "initial stock",
                });
            }
        }
        await db.SaveChangesAsync(ct);
    }

    /// <summary>Adds the robots that are missing (databases and snapshots created before the robot registry).</summary>
    public static async Task EnsureRobotsAsync(IWmsDbContext db, DateTime now, CancellationToken ct = default)
    {
        var existing = (await db.Robots.Select(r => r.Id).ToListAsync(ct)).ToHashSet();
        var missing = CellLayout.Robots.Where(d => !existing.Contains(d.Id)).ToList();
        if (missing.Count == 0) return;
        db.Robots.AddRange(missing.Select(d => new Robot
        {
            Id = d.Id, Model = d.Model, Role = d.Role, Station = d.Station, Sorter = d.Sorter, Tool = d.Tool, Controller = d.Controller,
            Mode = RobotMode.Auto, State = RobotState.Idle, StatsSince = now,
        }));
        await db.SaveChangesAsync(ct);
    }
}
