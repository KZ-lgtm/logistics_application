using Logistics.Wms.Application.Abstractions;
using Logistics.Wms.Application.Contracts;
using Logistics.Wms.Domain;
using Microsoft.EntityFrameworkCore;

namespace Logistics.Wms.Application.Services;

public sealed class OrderService(IWmsDbContext db, PutWallService putWall, TimeProvider clock)
{
    public const int MaxLinesPerOrder = 8;   // one order case: keep it to what fits in 0.35 × 0.30 × 0.22 m

    public async Task<OrderDto> CreateAsync(CreateOrderRequest request, CancellationToken ct = default)
    {
        if (string.IsNullOrWhiteSpace(request.Reference)) throw WmsException.Invalid("Reference is required.");
        if (request.Lines is null || request.Lines.Count == 0) throw WmsException.Invalid("An order needs at least one line.");
        if (request.Lines.Any(l => l.Quantity <= 0)) throw WmsException.Invalid("Line quantities must be positive.");
        var reference = request.Reference.Trim();
        if (await db.Orders.AnyAsync(o => o.Reference == reference, ct)) throw WmsException.Conflict($"Order reference '{reference}' already exists.");

        var merged = request.Lines.GroupBy(l => l.SkuCode.Trim(), StringComparer.OrdinalIgnoreCase)
            .Select(g => (Code: g.Key, Quantity: g.Sum(l => l.Quantity))).ToList();
        if (merged.Count > MaxLinesPerOrder) throw WmsException.Invalid($"An order may have at most {MaxLinesPerOrder} SKUs.");
        var codes = merged.Select(m => m.Code.ToUpperInvariant()).ToList();
        var skus = await db.Skus.Where(s => codes.Contains(s.Code.ToUpper())).ToListAsync(ct);
        var missing = codes.Except(skus.Select(s => s.Code.ToUpperInvariant())).ToList();
        if (missing.Count > 0) throw WmsException.Invalid($"Unknown SKU(s): {string.Join(", ", missing)}.");

        var order = new Order
        {
            Reference = reference, Priority = request.Priority, Status = OrderStatus.Queued, CreatedAt = clock.GetUtcNow().UtcDateTime,
            Lines = [.. merged.Select(m => new OrderLine
            {
                Sku = skus.Single(s => string.Equals(s.Code, m.Code, StringComparison.OrdinalIgnoreCase)), Quantity = m.Quantity,
            })],
        };
        db.Orders.Add(order);
        await db.SaveChangesAsync(ct);
        await putWall.ReleaseOrdersAsync(ct);
        await db.SaveChangesAsync(ct);
        return await GetAsync(order.Id, ct);
    }

    public async Task<OrderDto> GetAsync(int id, CancellationToken ct = default) =>
        (await Query().SingleOrDefaultAsync(o => o.Id == id, ct) ?? throw WmsException.NotFound("Order", id)).ToDto();

    public async Task<IReadOnlyList<OrderDto>> ListAsync(OrderStatus? status, int skip, int take, CancellationToken ct = default)
    {
        var q = Query();
        if (status is not null) q = q.Where(o => o.Status == status);
        var orders = await q.OrderByDescending(o => o.Id).Skip(Math.Max(0, skip)).Take(Math.Clamp(take, 1, 500)).ToListAsync(ct);
        return [.. orders.Select(o => o.ToDto())];
    }

    /// <summary>
    /// Cancels a queued order, or an active one as long as no carton has been picked for it (its allocations go back to the wave
    /// and its lane, with the still empty case, takes the next order). Once cartons are picked the order has to be finished.
    /// </summary>
    public async Task<OrderDto> CancelAsync(int id, CancellationToken ct = default)
    {
        var order = await Query().SingleOrDefaultAsync(o => o.Id == id, ct) ?? throw WmsException.NotFound("Order", id);
        switch (order.Status)
        {
            case OrderStatus.Queued:
                break;
            case OrderStatus.Active when order.Lines.All(l => l.Picked == 0):
                var now = clock.GetUtcNow().UtcDateTime;
                var pending = await db.PickTasks.Where(p => p.OrderId == id && p.Status == PickStatus.Pending).ToListAsync(ct);
                foreach (var p in pending) p.Status = PickStatus.Cancelled;
                foreach (var l in order.Lines) l.Allocated = 0;
                // bins at a stop whose remaining picks all belonged to this order leave the stop
                var binIds = pending.Select(p => p.BinId).Distinct().ToList();
                foreach (var bin in await db.Bins.Where(b => binIds.Contains(b.Id) && b.Location == BinLocation.AtStation).ToListAsync(ct))
                {
                    if (!await db.PickTasks.AnyAsync(p => p.BinId == bin.Id && p.Status == PickStatus.Pending && p.OrderId != id, ct))
                        PickingService.MoveToReturn(bin, now);
                }
                var lane = await db.Lanes.SingleAsync(l => l.Id == order.LaneId, ct);
                lane.OrderId = null;
                lane.State = LaneState.Ready;
                lane.StateSince = now;
                break;
            case OrderStatus.Active:
                throw WmsException.Conflict($"Order {id} already has picked cartons and cannot be cancelled.");
            default:
                throw WmsException.Conflict($"Order {id} is {order.Status}.");
        }
        order.Status = OrderStatus.Cancelled;
        order.LaneId = null;
        order.CompletedAt = clock.GetUtcNow().UtcDateTime;
        await db.SaveChangesAsync(ct);
        await putWall.ReleaseOrdersAsync(ct);
        await db.SaveChangesAsync(ct);
        return order.ToDto();
    }

    private IQueryable<Order> Query() => db.Orders.Include(o => o.Lines).ThenInclude(l => l.Sku);
}
