using Logistics.Wms.Application.Abstractions;

namespace Logistics.Wms.Api.Infrastructure;

/// <summary>Runs every state-changing request (anything but GET) under the cell's command gate.</summary>
public sealed class CommandGateFilter(WmsCommandGate gate) : IEndpointFilter
{
    public async ValueTask<object?> InvokeAsync(EndpointFilterInvocationContext context, EndpointFilterDelegate next)
    {
        if (HttpMethods.IsGet(context.HttpContext.Request.Method)) return await next(context);
        using var _ = await gate.EnterAsync(context.HttpContext.RequestAborted);
        return await next(context);
    }
}
