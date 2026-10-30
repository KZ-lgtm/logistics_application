using Logistics.Wms.Application.Abstractions;
using Logistics.Wms.Application.Contracts;
using Logistics.Wms.Domain;
using Microsoft.EntityFrameworkCore;

namespace Logistics.Wms.Application.Services;

/// <summary>Restock operator outside the west port: refill empty bins (or give an empty bin a new SKU) and send them back in.</summary>
public sealed class RestockService(IWmsDbContext db, TimeProvider clock)
{
    public async Task<IReadOnlyList<RestockBinDto>> ListAsync(CancellationToken ct = default)
    {
        var bins = await db.Bins.Include(b => b.Sku).Where(b => b.Location == BinLocation.RestockStation)
            .OrderBy(b => b.LastMovedAt).ThenBy(b => b.Id).ToListAsync(ct);
        return [.. bins.Select(b => new RestockBinDto(b.Id, b.Sku.Code, b.Quantity, b.Sku.BinCapacity, b.Sku.BinCapacity - b.Quantity, b.Sku.RestockBuffer))];
    }

    /// <summary>
    /// The operator presses "done": the bin now holds <see cref="RestockRequest.Quantity"/> cartons (default: full). Cartons waiting
    /// in the SKU's exception buffer are used first. An empty bin may be given another SKU (re-slotting of the assortment).
    /// </summary>
    public async Task<BinDto> CompleteAsync(string binId, RestockRequest request, CancellationToken ct = default)
    {
        var bin = await db.Bins.Include(b => b.Sku).Include(b => b.Slot).SingleOrDefaultAsync(b => b.Id == binId, ct)
            ?? throw WmsException.NotFound("Bin", binId);
        if (bin.Location != BinLocation.RestockStation) throw WmsException.Conflict($"Bin {binId} is not at the restock station ({bin.Location}).");

        if (request.SkuCode is { Length: > 0 } code && !string.Equals(code, bin.Sku.Code, StringComparison.OrdinalIgnoreCase))
        {
            if (!bin.IsEmpty) throw WmsException.Conflict($"Bin {binId} still holds {bin.Quantity} × {bin.Sku.Code}; only an empty bin can change SKU.");
            var sku = await db.Skus.SingleOrDefaultAsync(s => s.Code.ToUpper() == code.ToUpperInvariant(), ct) ?? throw WmsException.NotFound("SKU", code);
            bin.Sku = sku;
            bin.SkuId = sku.Id;
        }

        var target = request.Quantity ?? bin.Sku.BinCapacity;
        if (target < bin.Quantity || target > bin.Sku.BinCapacity)
            throw WmsException.Invalid($"Quantity must be between {bin.Quantity} and {bin.Sku.BinCapacity}.");

        var now = clock.GetUtcNow().UtcDateTime;
        var added = target - bin.Quantity;
        var fromBuffer = Math.Min(added, bin.Sku.RestockBuffer);
        bin.Quantity = target;
        bin.Sku.RestockBuffer -= fromBuffer;
        if (added > 0)
        {
            db.InventoryTransactions.Add(new InventoryTransaction
            {
                At = now, Type = InventoryTransactionType.Restock, SkuId = bin.SkuId, BinId = bin.Id, Quantity = added, BinQuantityAfter = target,
                Reference = fromBuffer > 0 ? $"restock, {fromBuffer} from the exception buffer" : "restock", User = request.User,
            });
        }
        if (fromBuffer > 0)
        {
            db.InventoryTransactions.Add(new InventoryTransaction
            {
                At = now, Type = InventoryTransactionType.Restock, SkuId = bin.SkuId, Quantity = -fromBuffer,
                Reference = $"exception buffer → bin {bin.Id}", User = request.User,
            });
        }

        bin.Location = BinLocation.RestockReturn;
        bin.LastMovedAt = now;
        await db.SaveChangesAsync(ct);
        return bin.ToDto();
    }
}
