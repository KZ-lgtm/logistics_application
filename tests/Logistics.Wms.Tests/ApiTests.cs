using System.Net;
using System.Net.Http.Json;
using System.Text.Json;
using System.Text.Json.Serialization;
using Logistics.Wms.Application.Contracts;
using Logistics.Wms.Domain;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.Mvc.Testing;

namespace Logistics.Wms.Tests;

public sealed class ApiFactory : WebApplicationFactory<Program>
{
    private readonly string _dbPath = Path.Combine(Path.GetTempPath(), $"wms-api-{Guid.NewGuid():N}.db");

    protected override void ConfigureWebHost(IWebHostBuilder builder)
    {
        builder.UseSetting("ConnectionStrings:Wms", $"Data Source={_dbPath};Pooling=False");
        builder.UseSetting("Simulation:Enabled", "false");
    }

    protected override void Dispose(bool disposing)
    {
        base.Dispose(disposing);
        foreach (var f in new[] { _dbPath, _dbPath + "-wal", _dbPath + "-shm" }) File.Delete(f);
    }
}

public class ApiTests(ApiFactory factory) : IClassFixture<ApiFactory>
{
    private static readonly JsonSerializerOptions Json = new(JsonSerializerDefaults.Web) { Converters = { new JsonStringEnumConverter() } };

    [Fact]
    public async Task Order_can_be_created_and_read()
    {
        var client = factory.CreateClient();
        var response = await client.PostAsJsonAsync("/api/orders", new CreateOrderRequest("API-1", [new("SKU-1007", 2)]), Json);
        Assert.Equal(HttpStatusCode.Created, response.StatusCode);
        var order = await response.Content.ReadFromJsonAsync<OrderDto>(Json);
        Assert.Contains(order!.Status, new[] { OrderStatus.Active, OrderStatus.Queued });   // the fixture database is shared

        var read = await client.GetFromJsonAsync<OrderDto>(response.Headers.Location, Json);
        Assert.Equal("API-1", read!.Reference);
    }

    [Theory]
    [InlineData("/api/orders/424242", HttpStatusCode.NotFound)]
    [InlineData("/api/inventory/bins/NOPE", HttpStatusCode.NotFound)]
    public async Task Unknown_resources_are_404_problems(string url, HttpStatusCode expected)
    {
        var response = await factory.CreateClient().GetAsync(url);
        Assert.Equal(expected, response.StatusCode);
        Assert.Equal("application/problem+json", response.Content.Headers.ContentType?.MediaType);
    }

    [Fact]
    public async Task Business_rule_violations_are_400_and_409()
    {
        var client = factory.CreateClient();
        var bad = await client.PostAsJsonAsync("/api/orders", new CreateOrderRequest("API-BAD", [new("SKU-0000", 1)]), Json);
        Assert.Equal(HttpStatusCode.BadRequest, bad.StatusCode);
        var conflict = await client.PostAsync("/api/putwall/lanes/S2-4/events/CaseReplaced", null);
        Assert.Equal(HttpStatusCode.Conflict, conflict.StatusCode);
    }

    [Fact]
    public async Task Robots_can_be_faulted_and_dispatched_over_http()
    {
        var client = factory.CreateClient();
        var robots = await client.GetFromJsonAsync<List<RobotDto>>("/api/robots", Json);
        Assert.Equal(5, robots!.Count);

        var fault = await client.PostAsJsonAsync("/api/robots/S2/fault", new RobotFaultRequest("E-417", "cup worn"), Json);
        Assert.Equal(RobotState.Fault, (await fault.Content.ReadFromJsonAsync<RobotDto>(Json))!.State);
        Assert.Equal(HttpStatusCode.NoContent, (await client.PostAsync("/api/robots/S2/tasks/next", null)).StatusCode);
        Assert.True((await client.PostAsync("/api/robots/S2/fault/clear", null)).IsSuccessStatusCode);
        Assert.Equal(HttpStatusCode.Conflict, (await client.PostAsync("/api/robots/S2/fault/clear", null)).StatusCode);
        Assert.Equal(HttpStatusCode.NotFound, (await client.GetAsync("/api/robots/R9")).StatusCode);
        var events = await client.GetFromJsonAsync<List<RobotEventDto>>("/api/robots/events?robot=S2", Json);
        Assert.Equal(2, events!.Count);
    }

    [Fact]
    public async Task Simulation_steps_show_up_on_the_dashboard()
    {
        var client = factory.CreateClient();
        Assert.True((await client.PostAsync("/api/simulation/step?rounds=60", null)).IsSuccessStatusCode);
        var dashboard = await client.GetFromJsonAsync<DashboardDto>("/api/dashboard", Json);
        Assert.Equal(8, dashboard!.Lanes.Count);
        Assert.True(dashboard.Throughput.ItemsPickedTotal > 0);
        Assert.Equal(2, dashboard.Stations.Count);
        Assert.Equal(5, dashboard.Robots.Count);
        Assert.True((await client.GetAsync("/openapi/v1.json")).IsSuccessStatusCode);
        Assert.Contains("Logistics WMS", await client.GetStringAsync("/"));
    }
}
