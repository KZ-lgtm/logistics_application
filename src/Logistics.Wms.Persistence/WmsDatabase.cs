using Logistics.Wms.Application.Services;
using Microsoft.EntityFrameworkCore;
using Microsoft.Extensions.DependencyInjection;

namespace Logistics.Wms.Persistence;

public static class WmsDatabase
{
    /// <summary>Creates the schema if needed and seeds the cell's master data and initial stock on an empty database.</summary>
    public static async Task InitializeAsync(IServiceProvider services, CancellationToken ct = default)
    {
        await using var scope = services.CreateAsyncScope();
        var db = scope.ServiceProvider.GetRequiredService<WmsDbContext>();
        await db.Database.EnsureCreatedAsync(ct);
        await WmsSeeder.SeedAsync(db, scope.ServiceProvider.GetRequiredService<TimeProvider>(), ct: ct);
    }
}
