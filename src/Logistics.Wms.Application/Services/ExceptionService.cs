using Logistics.Wms.Application.Abstractions;
using Logistics.Wms.Application.Contracts;
using Logistics.Wms.Domain;
using Microsoft.EntityFrameworkCore;

namespace Logistics.Wms.Application.Services;

/// <summary>Exception tote at the belt end, handled by the put-wall operator.</summary>
public sealed class ExceptionService(IWmsDbContext db, TimeProvider clock)
{
    public async Task<IReadOnlyList<ExceptionItemDto>> ListAsync(bool openOnly, int take, CancellationToken ct = default)
    {
        var q = db.ExceptionItems.Include(e => e.Sku).AsQueryable();
        if (openOnly) q = q.Where(e => e.Disposition == ExceptionDisposition.Open);
        var items = await q.OrderByDescending(e => e.Id).Take(Math.Clamp(take, 1, 500)).ToListAsync(ct);
        return [.. items.Select(e => e.ToDto())];
    }

    /// <summary>Return the carton to stock (it waits at the restock operator for the next bin of its SKU) or scrap it.</summary>
    public async Task<ExceptionItemDto> ResolveAsync(int id, ResolveExceptionRequest request, CancellationToken ct = default)
    {
        if (request.Disposition == ExceptionDisposition.Open) throw WmsException.Invalid("Choose ReturnedToStock or Scrapped.");
        var item = await db.ExceptionItems.Include(e => e.Sku).SingleOrDefaultAsync(e => e.Id == id, ct) ?? throw WmsException.NotFound("Exception", id);
        if (item.Disposition != ExceptionDisposition.Open) throw WmsException.Conflict($"Exception {id} is already {item.Disposition}.");

        var now = clock.GetUtcNow().UtcDateTime;
        item.Disposition = request.Disposition;
        item.ResolvedAt = now;
        item.ResolvedBy = request.User;
        if (request.Disposition == ExceptionDisposition.ReturnedToStock)
        {
            item.Sku.RestockBuffer++;
            db.InventoryTransactions.Add(new InventoryTransaction
            {
                At = now, Type = InventoryTransactionType.ReturnFromException, SkuId = item.SkuId, Quantity = 1,
                Reference = $"exception {item.Id} (pick {item.PickTaskId}) → restock buffer", User = request.User,
            });
        }
        await db.SaveChangesAsync(ct);
        return item.ToDto();
    }

    /// <summary>The operator emptied the tote; the belt may run cartons into it again.</summary>
    public async Task<int> EmptyToteAsync(CancellationToken ct = default)
    {
        var state = await db.CellState.SingleAsync(ct);
        var n = state.ExceptionToteCount;
        state.ExceptionToteCount = 0;
        await db.SaveChangesAsync(ct);
        return n;
    }
}
