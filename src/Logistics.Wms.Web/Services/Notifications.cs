namespace Logistics.Wms.Web.Services;

public sealed record Notification(int Id, string Text, bool IsError);

public sealed class Notifications
{
    private int _next;
    private readonly List<Notification> _items = [];

    public event Action? Changed;

    public IReadOnlyList<Notification> Items => _items;

    public void Info(string text) => Add(text, false);

    public void Error(string text) => Add(text, true);

    public void Dismiss(int id)
    {
        _items.RemoveAll(n => n.Id == id);
        Changed?.Invoke();
    }

    private void Add(string text, bool error)
    {
        var n = new Notification(++_next, text, error);
        _items.Add(n);
        if (_items.Count > 4) _items.RemoveAt(0);
        Changed?.Invoke();
        _ = Task.Delay(error ? 7000 : 3500).ContinueWith(_ => Dismiss(n.Id));
    }
}
