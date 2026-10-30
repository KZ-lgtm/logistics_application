using Logistics.Wms.Domain;
using Microsoft.EntityFrameworkCore;

namespace Logistics.Wms.Tests;

public class SeedTests
{
    [Fact]
    public async Task Seed_matches_the_cell_layout()
    {
        await using var h = await WmsHarness.CreateAsync();
        Assert.Equal(40, await h.Db.Skus.CountAsync());
        Assert.Equal(144, await h.Db.Slots.CountAsync());
        Assert.Equal(120, await h.Db.Bins.CountAsync());
        Assert.Equal(8, await h.Db.Lanes.CountAsync());
        Assert.False(await h.Db.Slots.AnyAsync(s => s.Column == 0 || s.Column == 13 || s.Column == 7));

        var bins = await h.Db.Bins.Include(b => b.Sku).Include(b => b.Slot).ToListAsync();
        Assert.All(bins, b =>
        {
            Assert.Equal(BinLocation.Rack, b.Location);
            Assert.Equal(b.Id, b.Slot!.BinId);
            Assert.InRange(b.Quantity, 1, b.Sku.BinCapacity);
        });
        Assert.Equal(bins.Sum(b => b.Quantity), await h.Db.InventoryTransactions.SumAsync(t => t.Quantity));
    }
}
