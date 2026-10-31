using System.Diagnostics;

namespace Logistics.Wms.Web.Services;

/// <summary>
/// The browser edition's clock. Normally it runs with real time; while the live 3D cell drives the WMS it follows the simulation
/// time instead (sim speed, pause), so cycle times, lead times and hourly figures match what the 3D cell shows. It never runs
/// backwards: switching modes and restoring a saved state continue from the latest time.
/// </summary>
public sealed class SimClock : TimeProvider
{
    private readonly Lock _lock = new();
    private DateTime _base = DateTime.UtcNow;
    private long _baseTicks = Stopwatch.GetTimestamp();
    private bool _followsSimulation;
    private double _simSeconds;

    public bool FollowsSimulation { get { lock (_lock) return _followsSimulation; } }

    public override DateTimeOffset GetUtcNow()
    {
        lock (_lock) return new DateTimeOffset(Current, TimeSpan.Zero);
    }

    private DateTime Current => _followsSimulation ? _base : _base + Stopwatch.GetElapsedTime(_baseTicks);

    /// <summary>The 3D cell reports its simulation time (seconds since its start); the clock advances by the difference.</summary>
    public void FollowSimulation(double simSeconds)
    {
        lock (_lock)
        {
            if (!_followsSimulation)
            {
                _base = Current;
                _followsSimulation = true;
            }
            else if (simSeconds > _simSeconds)
            {
                _base += TimeSpan.FromSeconds(simSeconds - _simSeconds);
            }
            _simSeconds = simSeconds;
        }
    }

    public void FollowRealTime()
    {
        lock (_lock)
        {
            _base = Current;
            _baseTicks = Stopwatch.GetTimestamp();
            _followsSimulation = false;
        }
    }

    /// <summary>Moves the clock forward to <paramref name="utc"/> if it is behind (a restored state saved at a later time).</summary>
    public void EnsureAtLeast(DateTime utc)
    {
        lock (_lock)
        {
            if (utc <= Current) return;
            _base = utc;
            _baseTicks = Stopwatch.GetTimestamp();
        }
    }
}
