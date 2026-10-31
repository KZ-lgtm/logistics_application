namespace Logistics.Wms.Web.Services;

/// <summary>
/// Lifetime of the live 3D cell. Once started it stays mounted in the layout and keeps running (and booking in the WMS) while other
/// pages are open, so orders created anywhere flow through it; only the Live 3D cell page shows it. Stopped by the user or a reload.
/// </summary>
public sealed class TwinService(SimulationRunner simulation, SimClock clock)
{
    public bool Started { get; private set; }
    /// <summary>Order the 3D view should point at (from "Watch in 3D").</summary>
    public int? FocusOrderId { get; private set; }

    public event Action? Changed;

    public void Start()
    {
        if (Started) return;
        Started = true;
        simulation.SetTwinActive(true);
        Changed?.Invoke();
    }

    public void Stop()
    {
        if (!Started) return;
        Started = false;
        FocusOrderId = null;
        simulation.SetTwinActive(false);
        clock.FollowRealTime();
        Changed?.Invoke();
    }

    public void Focus(int? orderId)
    {
        FocusOrderId = orderId;
        Changed?.Invoke();
    }
}
