using Logistics.Wms.Application.Abstractions;
using Logistics.Wms.Application.Services;
using Logistics.Wms.Application.Simulation;
using Microsoft.Extensions.DependencyInjection;

namespace Logistics.Wms.Application;

public static class DependencyInjection
{
    public static IServiceCollection AddWmsApplication(this IServiceCollection services, SimulationOptions? simulation = null)
    {
        services.AddSingleton(TimeProvider.System);
        services.AddSingleton<WmsCommandGate>();
        services.AddSingleton(new SimulationState(simulation ?? new SimulationOptions()));
        services.AddScoped<DemandService>();
        services.AddScoped<OrderService>();
        services.AddScoped<AsrsService>();
        services.AddScoped<PickingService>();
        services.AddScoped<SortationService>();
        services.AddScoped<PutWallService>();
        services.AddScoped<RestockService>();
        services.AddScoped<ExceptionService>();
        services.AddScoped<InventoryService>();
        services.AddScoped<DashboardService>();
        services.AddScoped<CellSimulator>();
        return services;
    }
}
