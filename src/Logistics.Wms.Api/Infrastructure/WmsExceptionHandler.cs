using Logistics.Wms.Domain;
using Microsoft.AspNetCore.Diagnostics;
using Microsoft.AspNetCore.Mvc;

namespace Logistics.Wms.Api.Infrastructure;

/// <summary>Maps business-rule violations to RFC 7807 problem details (404 / 409 / 400).</summary>
public sealed class WmsExceptionHandler(IProblemDetailsService problemDetails) : IExceptionHandler
{
    public async ValueTask<bool> TryHandleAsync(HttpContext context, Exception exception, CancellationToken ct)
    {
        if (exception is not WmsException wms) return false;
        var status = wms.Kind switch
        {
            WmsErrorKind.NotFound => StatusCodes.Status404NotFound,
            WmsErrorKind.Conflict => StatusCodes.Status409Conflict,
            _ => StatusCodes.Status400BadRequest,
        };
        context.Response.StatusCode = status;
        return await problemDetails.TryWriteAsync(new ProblemDetailsContext
        {
            HttpContext = context,
            Exception = exception,
            ProblemDetails = new ProblemDetails { Status = status, Title = wms.Kind.ToString(), Detail = wms.Message },
        });
    }
}
