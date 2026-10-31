using System.Diagnostics;
using Logistics.Wms.Application.Simulation;

namespace Logistics.Wms.Web.Services;

/// <summary>Runs the cell simulator in the browser: one round per tick while running, each round as one WMS command.</summary>
public sealed class SimulationRunner(WmsClient wms, SimulationState state)
{
    public SimulationState State => state;
    public bool Running => state.Running;
    /// <summary>The live 3D cell drives the WMS; the round-based simulator must not run at the same time.</summary>
    public bool TwinActive { get; private set; }
    public double LastRoundMs { get; private set; }

    public event Action? Changed;

    public void Start()
    {
        if (TwinActive) return;
        state.Running = true;
        Changed?.Invoke();
    }

    public void SetTwinActive(bool active)
    {
        TwinActive = active;
        if (active) state.Running = false;
        Changed?.Invoke();
    }

    public void Stop() { state.Running = false; Changed?.Invoke(); }

    public async Task StepAsync(int rounds = 1)
    {
        for (var i = 0; i < rounds; i++) await RoundAsync();
    }

    public async Task RunAsync(CancellationToken ct)
    {
        while (!ct.IsCancellationRequested)
        {
            await Task.Delay(Math.Max(20, state.Options.StepMilliseconds), ct);
            if (!state.Running) continue;
            if (!await RoundAsync()) Stop();
        }
    }

    private async Task<bool> RoundAsync()
    {
        var sw = Stopwatch.StartNew();
        var ok = await wms.CommandAsync(sp => sp.Get<CellSimulator>().StepAsync());
        LastRoundMs = sw.Elapsed.TotalMilliseconds;
        return ok;
    }
}
