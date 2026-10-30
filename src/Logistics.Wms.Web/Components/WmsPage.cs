using Logistics.Wms.Web.Services;
using Microsoft.AspNetCore.Components;

namespace Logistics.Wms.Web.Components;

/// <summary>Base for pages that show WMS data: loads on start and reloads (throttled) after every change.</summary>
public abstract class WmsPage : ComponentBase, IDisposable
{
    private bool _reloadPending;

    [Inject] protected WmsClient Wms { get; set; } = null!;

    /// <summary>Minimum time between two reloads while the cell is busy.</summary>
    protected virtual TimeSpan Throttle => TimeSpan.FromMilliseconds(450);

    protected abstract Task LoadAsync();

    protected override async Task OnInitializedAsync()
    {
        Wms.Changed += OnChanged;
        await LoadAsync();
    }

    protected Task ReloadAsync() => LoadAsync();

    private void OnChanged()
    {
        if (_reloadPending) return;
        _reloadPending = true;
        _ = InvokeAsync(async () =>
        {
            await Task.Delay(Throttle);
            _reloadPending = false;
            await LoadAsync();
            StateHasChanged();
        });
    }

    public virtual void Dispose()
    {
        Wms.Changed -= OnChanged;
        GC.SuppressFinalize(this);
    }
}
