using Logistics.Wms.Application.Abstractions;
using Logistics.Wms.Application.Simulation;

namespace Logistics.Wms.Api.Infrastructure;

/// <summary>Drives the cell simulator in the background while it is running (config "Simulation:Enabled" or POST /api/simulation/start).</summary>
public sealed class SimulationHostedService(IServiceScopeFactory scopes, SimulationState state, WmsCommandGate gate, ILogger<SimulationHostedService> log)
    : BackgroundService
{
    protected override async Task ExecuteAsync(CancellationToken stoppingToken)
    {
        while (!stoppingToken.IsCancellationRequested)
        {
            if (state.Running)
            {
                try
                {
                    await using var scope = scopes.CreateAsyncScope();
                    using var _ = await gate.EnterAsync(stoppingToken);
                    await scope.ServiceProvider.GetRequiredService<CellSimulator>().StepAsync(stoppingToken);
                }
                catch (OperationCanceledException) when (stoppingToken.IsCancellationRequested)
                {
                    break;
                }
                catch (Exception ex)
                {
                    log.LogError(ex, "Simulation round {Round} failed; simulation stopped.", state.Rounds);
                    state.Running = false;
                }
            }
            try
            {
                await Task.Delay(Math.Max(10, state.Options.StepMilliseconds), stoppingToken);
            }
            catch (OperationCanceledException)
            {
                break;
            }
        }
    }
}
