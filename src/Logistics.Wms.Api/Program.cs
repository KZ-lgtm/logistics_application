using System.Text.Json.Serialization;
using Logistics.Wms.Api.Endpoints;
using Logistics.Wms.Api.Infrastructure;
using Logistics.Wms.Application;
using Logistics.Wms.Application.Simulation;
using Logistics.Wms.Infrastructure;

var builder = WebApplication.CreateBuilder(args);

var simulation = builder.Configuration.GetSection("Simulation").Get<SimulationOptions>() ?? new SimulationOptions();
builder.Services.AddWmsApplication(simulation);
builder.Services.AddWmsInfrastructure(builder.Configuration.GetConnectionString("Wms") ?? "Data Source=wms.db");
builder.Services.AddHostedService<SimulationHostedService>();
builder.Services.ConfigureHttpJsonOptions(o => o.SerializerOptions.Converters.Add(new JsonStringEnumConverter()));
builder.Services.AddProblemDetails();
builder.Services.AddExceptionHandler<WmsExceptionHandler>();
builder.Services.AddOpenApi();

var app = builder.Build();

await app.Services.InitializeWmsDatabaseAsync();

app.UseExceptionHandler();
app.UseStatusCodePages();
app.UseDefaultFiles();
app.UseStaticFiles();
app.MapOpenApi();

var api = app.MapGroup("/api").AddEndpointFilter<CommandGateFilter>();
api.MapOrderEndpoints();
api.MapAsrsEndpoints();
api.MapRobotEndpoints();
api.MapPickingEndpoints();
api.MapSortationEndpoints();
api.MapPutWallEndpoints();
api.MapRestockEndpoints();
api.MapInventoryEndpoints();
api.MapExceptionEndpoints();
api.MapDashboardEndpoints();
api.MapSimulationEndpoints();

app.Run();

public partial class Program;
