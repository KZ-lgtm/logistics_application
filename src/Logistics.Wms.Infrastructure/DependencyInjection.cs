using Logistics.Wms.Application.Abstractions;
using Logistics.Wms.Persistence;
using Microsoft.EntityFrameworkCore;
using Microsoft.Extensions.DependencyInjection;

namespace Logistics.Wms.Infrastructure;

public static class DependencyInjection
{
    public static IServiceCollection AddWmsInfrastructure(this IServiceCollection services, string connectionString)
    {
        services.AddDbContext<WmsDbContext>(o => o.UseSqlite(connectionString));
        services.AddScoped<IWmsDbContext>(sp => sp.GetRequiredService<WmsDbContext>());
        return services;
    }

    /// <summary>Creates the schema if needed, switches SQLite to WAL and seeds an empty database.</summary>
    public static async Task InitializeWmsDatabaseAsync(this IServiceProvider services, CancellationToken ct = default)
    {
        await using (var scope = services.CreateAsyncScope())
        {
            var db = scope.ServiceProvider.GetRequiredService<WmsDbContext>();
            await db.Database.EnsureCreatedAsync(ct);
            await db.Database.ExecuteSqlRawAsync("PRAGMA journal_mode=WAL;", ct);
        }
        await WmsDatabase.InitializeAsync(services, ct);
    }
}
