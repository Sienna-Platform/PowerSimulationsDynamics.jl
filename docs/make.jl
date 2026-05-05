using Documenter, PowerSystems, DocStringExtensions, PowerSimulationsDynamics
using DocumenterInterLinks

include(joinpath(@__DIR__, "make_tutorials.jl"))
make_tutorials()

links = InterLinks(
    "PowerSystems" => "https://sienna-platform.github.io/PowerSystems.jl/stable/",
    "PowerSystemCaseBuilder" => "https://nrel-sienna.github.io/PowerSystemCaseBuilder.jl/stable/",
)

makedocs(;
    modules = [PowerSimulationsDynamics],
    format = Documenter.HTML(;
        prettyurls = haskey(ENV, "GITHUB_ACTIONS"),
        mathengine = Documenter.MathJax(),
    ),
    sitename = "PowerSimulationsDynamics.jl",
    plugins = [links],
    pages = Any[
        "Welcome Page" => "index.md",
        "Quick Start Guide" => "tutorials/generated_quick_start_guide.md",
        "Simulation Execution" => "execute.md",
        "Tutorials" => Any[
            "Create Dynamic Data" => "tutorials/generated_tutorial_dynamic_data.md",
            "OMIB" => "tutorials/generated_tutorial_omib.md",
            "Dynamic Lines Simulation" => "tutorials/generated_tutorial_dynamic_lines.md",
            "Inverter Modeling" => "tutorials/generated_tutorial_inverter_modeling.md",
            "240 WECC solver comparison" => "tutorials/generated_tutorial_240bus.md",
            "Small-Signal Analysis" => "tutorials/generated_tutorial_continuation_pf.md",
            "Active Load Model" => "tutorials/generated_tutorial_activeload.md",
        ],
        "Models" => "models.md",
        "Initialization" => "initialization.md",
        "Small Signal" => "small.md",
        "Reference Frames" => "reference_frames.md",
        "Perturbations" => "perturbations.md",
        "Time Delays" => "time_delays.md",
        "Industrial Renewable Models" => "generic.md",
        "Generator Component Library" => Any[
            "Machine" => "component_models/machines.md",
            "Shaft" => "component_models/shafts.md",
            "AVR" => "component_models/avr.md",
            "PSS" => "component_models/pss.md",
            "Turbine and Governor" => "component_models/turbine_gov.md",
        ],
        "CIG Component Library" => Any[
            "Converter" => "component_models/converter.md",
            "DC Sources" => "component_models/dc_source.md",
            "Filter" => "component_models/filters.md",
            "Frequency Estimators" => "component_models/freq_esti.md",
            "Inner Control" => "component_models/inner_control.md",
            "Outer Control" => "component_models/outer_control.md",
        ],
        "Branch Models" => Any["Network" => "component_models/network.md",],
        "Load Models" => Any["Load Models" => "component_models/loads.md",],
        "Code Base Developer Guide" => Any["Developer Guide" => "code_base_developer_guide/developer.md",],
        "Public API Reference" => "api/public.md",
        "Internal API Reference" => "api/internal.md",
    ],
)

deploydocs(;
    repo = "github.com/Sienna-Platform/PowerSimulationsDynamics.jl.git",
    target = "build",
    branch = "gh-pages",
    devbranch = "main",
    devurl = "dev",
    push_preview = true,
    versions = ["stable" => "v^", "v#.#"],
)
