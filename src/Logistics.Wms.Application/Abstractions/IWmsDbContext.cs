using Logistics.Wms.Domain;
using Microsoft.EntityFrameworkCore;

namespace Logistics.Wms.Application.Abstractions;

public interface IWmsDbContext
{
    DbSet<Sku> Skus { get; }
    DbSet<StorageSlot> Slots { get; }
    DbSet<Bin> Bins { get; }
    DbSet<Order> Orders { get; }
    DbSet<OrderLine> OrderLines { get; }
    DbSet<PutWallLane> Lanes { get; }
    DbSet<AsrsTask> AsrsTasks { get; }
    DbSet<PickTask> PickTasks { get; }
    DbSet<ExceptionItem> ExceptionItems { get; }
    DbSet<InventoryTransaction> InventoryTransactions { get; }
    DbSet<CellState> CellState { get; }

    Task<int> SaveChangesAsync(CancellationToken cancellationToken = default);
}
