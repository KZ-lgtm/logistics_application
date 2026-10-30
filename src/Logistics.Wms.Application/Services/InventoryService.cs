using Logistics.Wms.Application.Abstractions;
using Logistics.Wms.Application.Contracts;
using Logistics.Wms.Domain;
using Microsoft.EntityFrameworkCore;

namespace Logistics.Wms.Application.Services;

public sealed class InventoryService(IWmsDbContext db, DemandService demand, TimeProvider clock)
{
    public async Task<IReadOnlyList<SkuStockDto>> GetStockAsync(CancellationToken ct = default)
    {
        var skus = await db.Skus.OrderBy(s => s.Code).ToListAsync(ct);
        var bins = await db.Bins.Select(b => new { b.SkuId, b.Quantity, b.Location }).ToListAsync(ct);
        var pending = await db.PickTasks.Where(p => p.Status == PickStatus.Pending).GroupBy(p => p.SkuId)
            .Select(g => new { g.Key, N = g.Count() }).ToDictionaryAsync(x => x.Key, x => x.N, ct);
        var open = await demand.OpenQuantityBySkuAsync(ct);

        return [.. skus.Select(s =>
        {
            var mine = bins.Where(b => b.SkuId == s.Id).ToList();
            var inRack = mine.Where(b => b.Location == BinLocation.Rack).Sum(b => b.Quantity);
            var total = mine.Sum(b => b.Quantity) + s.RestockBuffer;
            return new SkuStockDto(s.Id, s.Code, s.Name, s.Size, s.BinCapacity, mine.Count, inRack, mine.Sum(b => b.Quantity) - inRack, total,
                pending.GetValueOrDefault(s.Id), open.GetValueOrDefault(s.Id), s.RestockBuffer, s.ReorderPoint, total <= s.ReorderPoint);
        })];
    }

    public async Task<SkuStockDto> CreateSkuAsync(CreateSkuRequest request, CancellationToken ct = default)
    {
        var code = request.Code?.Trim().ToUpperInvariant();
        if (string.IsNullOrEmpty(code)) throw WmsException.Invalid("Code is required.");
        if (string.IsNullOrWhiteSpace(request.Name)) throw WmsException.Invalid("Name is required.");
        if (request.WeightKg <= 0 || request.WeightKg > 2.0) throw WmsException.Invalid("Weight must be in (0, 2] kg (vacuum gripper payload).");
        if (request.ReorderPoint < 0) throw WmsException.Invalid("Reorder point must not be negative.");
        if (await db.Skus.AnyAsync(s => s.Code == code, ct)) throw WmsException.Conflict($"SKU '{code}' already exists.");
        db.Skus.Add(new Sku { Code = code, Name = request.Name.Trim(), Size = request.Size, WeightKg = request.WeightKg, ReorderPoint = request.ReorderPoint });
        await db.SaveChangesAsync(ct);
        return (await GetStockAsync(ct)).Single(s => s.Code == code);
    }

    public async Task<IReadOnlyList<BinDto>> ListBinsAsync(string? skuCode, BinLocation? location, CancellationToken ct = default)
    {
        var q = db.Bins.Include(b => b.Sku).Include(b => b.Slot).AsQueryable();
        if (!string.IsNullOrWhiteSpace(skuCode)) q = q.Where(b => b.Sku.Code.ToUpper() == skuCode.Trim().ToUpper());
        if (location is not null) q = q.Where(b => b.Location == location);
        return [.. (await q.OrderBy(b => b.Id).ToListAsync(ct)).Select(b => b.ToDto())];
    }

    public async Task<BinDto> GetBinAsync(string id, CancellationToken ct = default) =>
        (await db.Bins.Include(b => b.Sku).Include(b => b.Slot).SingleOrDefaultAsync(b => b.Id == id, ct) ?? throw WmsException.NotFound("Bin", id)).ToDto();

    /// <summary>Cycle count of a bin in the rack (not moving) or at the restock station.</summary>
    public async Task<BinDto> AdjustBinAsync(string id, AdjustBinRequest request, CancellationToken ct = default)
    {
        var bin = await db.Bins.Include(b => b.Sku).Include(b => b.Slot).SingleOrDefaultAsync(b => b.Id == id, ct) ?? throw WmsException.NotFound("Bin", id);
        if (!(bin.Location == BinLocation.Rack && !bin.Reserved) && bin.Location != BinLocation.RestockStation)
            throw WmsException.Conflict($"Bin {id} is moving ({bin.Location}); count it at a picking stop or when it is stored.");
        if (request.CountedQuantity < 0 || request.CountedQuantity > bin.Sku.BinCapacity)
            throw WmsException.Invalid($"Counted quantity must be between 0 and {bin.Sku.BinCapacity}.");

        var delta = request.CountedQuantity - bin.Quantity;
        if (delta != 0)
        {
            bin.Quantity = request.CountedQuantity;
            db.InventoryTransactions.Add(new InventoryTransaction
            {
                At = clock.GetUtcNow().UtcDateTime, Type = InventoryTransactionType.Adjustment, SkuId = bin.SkuId, BinId = bin.Id,
                Quantity = delta, BinQuantityAfter = bin.Quantity, Reference = request.Reason ?? "cycle count", User = request.User,
            });
            await db.SaveChangesAsync(ct);
        }
        return bin.ToDto();
    }

    public async Task<IReadOnlyList<SlotDto>> ListSlotsAsync(CancellationToken ct = default) =>
        [.. (await db.Slots.OrderBy(s => s.Column).ThenBy(s => s.Level).ToListAsync(ct)).Select(s => s.ToDto())];

    /// <summary>Blocks a free slot for maintenance (or releases it); dynamic slotting skips blocked slots.</summary>
    public async Task<SlotDto> SetSlotBlockedAsync(int id, bool blocked, CancellationToken ct = default)
    {
        var slot = await db.Slots.SingleOrDefaultAsync(s => s.Id == id, ct) ?? throw WmsException.NotFound("Slot", id);
        if (blocked && (slot.BinId is not null || slot.Reserved)) throw WmsException.Conflict($"Slot {slot.Code} is in use.");
        slot.Blocked = blocked;
        await db.SaveChangesAsync(ct);
        return slot.ToDto();
    }

    public async Task<IReadOnlyList<InventoryTransactionDto>> ListTransactionsAsync(string? skuCode, string? binId, int take, CancellationToken ct = default)
    {
        var q = from t in db.InventoryTransactions join s in db.Skus on t.SkuId equals s.Id select new { t, s.Code };
        if (!string.IsNullOrWhiteSpace(skuCode)) q = q.Where(x => x.Code.ToUpper() == skuCode.Trim().ToUpper());
        if (!string.IsNullOrWhiteSpace(binId)) q = q.Where(x => x.t.BinId == binId);
        return await q.OrderByDescending(x => x.t.Id).Take(Math.Clamp(take, 1, 1000))
            .Select(x => new InventoryTransactionDto(x.t.Id, x.t.At, x.t.Type, x.Code, x.t.BinId, x.t.Quantity, x.t.BinQuantityAfter, x.t.Reference, x.t.User))
            .ToListAsync(ct);
    }
}
