using Logistics.Wms.Application.Abstractions;
using Logistics.Wms.Domain;
using Microsoft.Extensions.DependencyInjection;

namespace Logistics.Wms.Web.Services;

/// <summary>
/// The browser edition's entry point into the WMS services. Every call runs in its own DI scope (fresh DbContext, so the change
/// tracker never grows); commands also run under the cell's <see cref="WmsCommandGate"/>, mark the state for saving and notify the
/// pages. Business-rule violations become a notification instead of an exception.
/// </summary>
public sealed class WmsClient(IServiceScopeFactory scopes, WmsCommandGate gate, Notifications notifications)
{
    /// <summary>Raised after every successful command.</summary>
    public event Action? Changed;

    public bool Dirty { get; set; }

    public async Task<T> QueryAsync<T>(Func<IServiceProvider, Task<T>> query)
    {
        await using var scope = scopes.CreateAsyncScope();
        return await query(scope.ServiceProvider);
    }

    /// <summary>Runs a command; returns its result, or default when it was refused (the reason is shown as a notification).</summary>
    public async Task<T?> CommandAsync<T>(Func<IServiceProvider, Task<T>> command, string? success = null)
    {
        try
        {
            T result;
            using (await gate.EnterAsync())
            {
                await using var scope = scopes.CreateAsyncScope();
                result = await command(scope.ServiceProvider);
            }
            Dirty = true;
            if (success is not null) notifications.Info(success);
            Changed?.Invoke();
            return result;
        }
        catch (WmsException ex)
        {
            notifications.Error(ex.Message);
            return default;
        }
    }

    /// <summary>Runs a command for the live 3D cell: refusals come back as an error text instead of a notification.</summary>
    public async Task<(bool Ok, string? Error, T? Value)> TryCommandAsync<T>(Func<IServiceProvider, Task<T>> command)
    {
        try
        {
            T result;
            using (await gate.EnterAsync())
            {
                await using var scope = scopes.CreateAsyncScope();
                result = await command(scope.ServiceProvider);
            }
            Dirty = true;
            Changed?.Invoke();
            return (true, null, result);
        }
        catch (WmsException ex)
        {
            return (false, ex.Message, default);
        }
    }

    public Task<bool> CommandAsync(Func<IServiceProvider, Task> command, string? success = null) =>
        CommandAsync(async sp => { await command(sp); return true; }, success).ContinueWith(t => t.Result);

    /// <summary>Lets pages refresh after a change that bypassed <see cref="CommandAsync{T}"/> (restore, reset).</summary>
    public void NotifyChanged() => Changed?.Invoke();

    /// <summary>Exclusive access for whole-database operations (snapshot, restore, compaction).</summary>
    public Task<IDisposable> LockAsync() => gate.EnterAsync();
}

public static class ServiceProviderExtensions
{
    public static T Get<T>(this IServiceProvider sp) where T : notnull => sp.GetRequiredService<T>();
}
