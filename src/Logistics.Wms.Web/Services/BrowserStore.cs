using System.Diagnostics;
using Logistics.Wms.Application.Services;
using Logistics.Wms.Persistence;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.JSInterop;

namespace Logistics.Wms.Web.Services;

/// <summary>
/// Keeps the in-memory WMS database in the browser's local storage: restored on start, saved a few seconds after a change.
/// History is compacted automatically before the snapshot outgrows what local storage reliably holds.
/// </summary>
public sealed class BrowserStore(IServiceProvider services, IServiceScopeFactory scopes, WmsClient wms, IJSRuntime js, SimClock clock, Notifications notifications)
{
    // the github.io origin is shared by all of an account's Pages sites, so the key carries the application's name
    public const string Key = "logistics-wms/v1/snapshot";
    // the in-memory provider has no indexes, so the operational tables are kept small; compaction keeps the newest 300 orders
    private const int CompactAboveChars = 2_000_000;
    private const int CompactAboveOrders = 600;

    public DateTime? LastSaved { get; private set; }
    public int LastSize { get; private set; }
    public double LastSaveMs { get; private set; }
    public string? Warning { get; private set; }
    /// <summary>True when the current state came from local storage rather than a fresh seed.</summary>
    public bool Restored { get; private set; }

    /// <summary>Restores the saved state, or seeds a new cell. Returns true when a saved state was found.</summary>
    public async Task<bool> LoadAsync()
    {
        string? json = null;
        try { json = await js.InvokeAsync<string?>("localStorage.getItem", Key); }
        catch (JSException) { Warning = Localizer.Current?["Local storage is not available; changes are lost when the page closes."] ?? "Local storage is not available; changes are lost when the page closes."; }

        if (json is not null)
        {
            try
            {
                await RestoreAsync(json);
                LastSize = json.Length;
                Restored = true;
                return true;
            }
            catch (Exception ex)
            {
                notifications.Error((Localizer.Current?.F("The saved state could not be read ({0}); starting with a new cell.", ex.Message))
                    ?? $"The saved state could not be read ({ex.Message}); starting with a new cell.");
            }
        }
        await ResetAsync(save: false);
        return false;
    }

    public async Task RestoreAsync(string json)
    {
        var snapshot = WmsSnapshot.FromJson(json);
        clock.EnsureAtLeast(snapshot.SavedAt);
        using (await wms.LockAsync())
        {
            await using var scope = scopes.CreateAsyncScope();
            await snapshot.RestoreAsync(scope.ServiceProvider.GetRequiredService<WmsDbContext>());
        }
        wms.Dirty = true;
        wms.NotifyChanged();
    }

    /// <summary>Back to the initial cell: master data and seeded stock, no orders.</summary>
    public async Task ResetAsync(bool save = true)
    {
        using (await wms.LockAsync())
        {
            await using (var scope = scopes.CreateAsyncScope())
                await scope.ServiceProvider.GetRequiredService<WmsDbContext>().Database.EnsureDeletedAsync();
            await WmsDatabase.InitializeAsync(services);
        }
        wms.Dirty = save;
        wms.NotifyChanged();
    }

    public async Task<string> ExportAsync()
    {
        using (await wms.LockAsync())
        {
            await using var scope = scopes.CreateAsyncScope();
            return (await WmsSnapshot.CaptureAsync(scope.ServiceProvider.GetRequiredService<WmsDbContext>(), clock.GetUtcNow().UtcDateTime)).ToJson();
        }
    }

    public async Task<CompactionResult> CompactAsync()
    {
        CompactionResult result;
        using (await wms.LockAsync())
        {
            await using var scope = scopes.CreateAsyncScope();
            result = await scope.ServiceProvider.GetRequiredService<HistoryService>().CompactAsync();
        }
        wms.Dirty = true;
        wms.NotifyChanged();
        return result;
    }

    public async Task SaveAsync()
    {
        wms.Dirty = false;
        var sw = Stopwatch.StartNew();
        var json = await ExportAsync();
        int orders;
        await using (var scope = scopes.CreateAsyncScope())
            orders = scope.ServiceProvider.GetRequiredService<WmsDbContext>().Orders.Count();
        if (json.Length > CompactAboveChars || orders > CompactAboveOrders)
        {
            await CompactAsync();
            wms.Dirty = false;
            json = await ExportAsync();
        }
        try
        {
            await js.InvokeVoidAsync("localStorage.setItem", Key, json);
            Warning = null;
        }
        catch (JSException ex)
        {
            Warning = Localizer.Current?.F("Saving to local storage failed ({0}). Export a backup or compact the history.", ex.Message.Split('\n')[0])
                ?? $"Saving to local storage failed ({ex.Message.Split('\n')[0]}). Export a backup or compact the history.";
        }
        LastSaved = clock.GetUtcNow().UtcDateTime;
        LastSize = json.Length;
        LastSaveMs = sw.Elapsed.TotalMilliseconds;
    }

    public async Task ClearAsync()
    {
        try { await js.InvokeVoidAsync("localStorage.removeItem", Key); } catch (JSException) { }
    }

    /// <summary>Saves whenever something changed, at most every <paramref name="interval"/>.</summary>
    public async Task RunAutosaveAsync(TimeSpan interval, CancellationToken ct)
    {
        while (!ct.IsCancellationRequested)
        {
            await Task.Delay(interval, ct);
            if (!wms.Dirty) continue;
            try { await SaveAsync(); }
            catch (Exception ex) { Warning = $"Autosave failed: {ex.Message}"; }
        }
    }
}
