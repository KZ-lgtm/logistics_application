namespace Logistics.Wms.Application.Abstractions;

/// <summary>
/// Serialises all state-changing WMS commands of the cell. Decisions such as "which bin does R1 fetch next" read and then change
/// shared state, so two commands must never interleave. One cell produces a few commands per second, so a single gate is plenty.
/// Enter it once at the outer boundary (API request, simulator step); it is not re-entrant.
/// </summary>
public sealed class WmsCommandGate
{
    private readonly SemaphoreSlim _semaphore = new(1, 1);

    public async Task<IDisposable> EnterAsync(CancellationToken cancellationToken = default)
    {
        await _semaphore.WaitAsync(cancellationToken);
        return new Releaser(_semaphore);
    }

    private sealed class Releaser(SemaphoreSlim semaphore) : IDisposable
    {
        private int _disposed;

        public void Dispose()
        {
            if (Interlocked.Exchange(ref _disposed, 1) == 0) semaphore.Release();
        }
    }
}
