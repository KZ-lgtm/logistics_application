using Logistics.Wms.Application.Abstractions;
using Logistics.Wms.Domain;
using Microsoft.EntityFrameworkCore;
using Microsoft.EntityFrameworkCore.Storage.ValueConversion;

namespace Logistics.Wms.Infrastructure.Persistence;

public sealed class WmsDbContext(DbContextOptions<WmsDbContext> options) : DbContext(options), IWmsDbContext
{
    public DbSet<Sku> Skus => Set<Sku>();
    public DbSet<StorageSlot> Slots => Set<StorageSlot>();
    public DbSet<Bin> Bins => Set<Bin>();
    public DbSet<Order> Orders => Set<Order>();
    public DbSet<OrderLine> OrderLines => Set<OrderLine>();
    public DbSet<PutWallLane> Lanes => Set<PutWallLane>();
    public DbSet<AsrsTask> AsrsTasks => Set<AsrsTask>();
    public DbSet<PickTask> PickTasks => Set<PickTask>();
    public DbSet<ExceptionItem> ExceptionItems => Set<ExceptionItem>();
    public DbSet<InventoryTransaction> InventoryTransactions => Set<InventoryTransaction>();
    public DbSet<CellState> CellState => Set<CellState>();

    protected override void ConfigureConventions(ModelConfigurationBuilder builder)
    {
        builder.Properties<Enum>().HaveConversion<string>().HaveMaxLength(32);
        builder.Properties<DateTime>().HaveConversion<UtcDateTimeConverter>();
        builder.Properties<DateTime?>().HaveConversion<UtcDateTimeConverter>();
    }

    /// <summary>All times are stored as UTC; SQLite drops the kind, so it is restored on read.</summary>
    private sealed class UtcDateTimeConverter() : ValueConverter<DateTime, DateTime>(
        v => v.Kind == DateTimeKind.Utc ? v : v.ToUniversalTime(),
        v => DateTime.SpecifyKind(v, DateTimeKind.Utc));

    protected override void OnModelCreating(ModelBuilder b)
    {
        b.Entity<Sku>(e =>
        {
            e.Property(x => x.Code).HasMaxLength(32);
            e.HasIndex(x => x.Code).IsUnique();
            e.Property(x => x.Name).HasMaxLength(200);
            e.Ignore(x => x.BinCapacity);
        });

        b.Entity<StorageSlot>(e =>
        {
            e.ToTable("Slots");
            e.HasIndex(x => new { x.Column, x.Level }).IsUnique();
            e.HasIndex(x => x.BinId).IsUnique();
            e.Property(x => x.BinId).HasMaxLength(16);
            e.Ignore(x => x.Code);
            e.Ignore(x => x.IsFree);
        });

        b.Entity<Bin>(e =>
        {
            e.Property(x => x.Id).HasMaxLength(16);
            e.HasOne(x => x.Sku).WithMany().HasForeignKey(x => x.SkuId).OnDelete(DeleteBehavior.Restrict);
            e.HasOne(x => x.Slot).WithMany().HasForeignKey(x => x.SlotId).OnDelete(DeleteBehavior.Restrict);
            e.HasIndex(x => x.Location);
            e.HasIndex(x => x.SkuId);
            e.Ignore(x => x.IsEmpty);
            e.Ignore(x => x.IsOnPickLoop);
            e.Ignore(x => x.IsOnRestockLoop);
        });

        b.Entity<Order>(e =>
        {
            e.Property(x => x.Reference).HasMaxLength(64);
            e.HasIndex(x => x.Reference).IsUnique();
            e.HasIndex(x => x.Status);
            e.Property(x => x.LaneId).HasMaxLength(8);
            e.HasMany(x => x.Lines).WithOne(x => x.Order).HasForeignKey(x => x.OrderId).OnDelete(DeleteBehavior.Cascade);
            e.Ignore(x => x.TotalQuantity);
            e.Ignore(x => x.IsFullySorted);
        });

        b.Entity<OrderLine>(e =>
        {
            e.HasOne(x => x.Sku).WithMany().HasForeignKey(x => x.SkuId).OnDelete(DeleteBehavior.Restrict);
            e.HasIndex(x => x.SkuId);
            e.Ignore(x => x.OpenQuantity);
        });

        b.Entity<PutWallLane>(e =>
        {
            e.ToTable("Lanes");
            e.Property(x => x.Id).HasMaxLength(8);
        });

        b.Entity<AsrsTask>(e =>
        {
            e.Property(x => x.BinId).HasMaxLength(16);
            e.HasIndex(x => x.Status);
        });

        b.Entity<PickTask>(e =>
        {
            e.Property(x => x.BinId).HasMaxLength(16);
            e.Property(x => x.TargetLaneId).HasMaxLength(8);
            e.HasOne(x => x.OrderLine).WithMany().HasForeignKey(x => x.OrderLineId).OnDelete(DeleteBehavior.Restrict);
            e.HasIndex(x => x.Status);
            e.HasIndex(x => new { x.BinId, x.Status });
            e.HasIndex(x => x.OrderId);
        });

        b.Entity<ExceptionItem>(e =>
        {
            e.HasOne(x => x.Sku).WithMany().HasForeignKey(x => x.SkuId).OnDelete(DeleteBehavior.Restrict);
            e.Property(x => x.ReadBarcode).HasMaxLength(64);
            e.Property(x => x.ResolvedBy).HasMaxLength(64);
            e.HasIndex(x => x.Disposition);
        });

        b.Entity<InventoryTransaction>(e =>
        {
            e.Property(x => x.BinId).HasMaxLength(16);
            e.Property(x => x.Reference).HasMaxLength(200);
            e.Property(x => x.User).HasMaxLength(64);
            e.HasIndex(x => x.SkuId);
            e.HasIndex(x => x.BinId);
        });

        b.Entity<CellState>(e => e.Property(x => x.Id).ValueGeneratedNever());
    }
}
