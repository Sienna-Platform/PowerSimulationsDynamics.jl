using Documenter, PowerSystems, DocStringExtensions, PowerSimulationsDynamics
using Literate

# UPDATE FOR CURRENT MODULE NAME HERE
const _DOCS_BASE_URL = "https://nrel-sienna.github.io/PowerSimulationsDynamics.jl/stable"

include(joinpath(@__DIR__, "make_tutorials.jl"))
make_tutorials()

makedocs(;
    modules = [PowerSimulationsDynamics],
    format = Documenter.HTML(; mathengine = Documenter.MathJax()),
    sitename = "PowerSimulationsDynamics.jl",
    pages = Any[
        "Welcome Page" => "index.md",
        "Quick Start Guide" => "quick_start_guide.md",
        "Simulation Execution" => "execute.md",
        "Tutorials" => Any[
            "Create Dynamic Data" => "tutorials/tutorial_dynamic_data.md",
            "OMIB" => "tutorials/tutorial_omib.md",
            "Dynamic Lines Simulation" => "tutorials/tutorial_dynamic_lines.md",
            "Inverter Modeling" => "tutorials/tutorial_inverter_modeling.md",
            "240 WECC solver comparison" => "tutorials/tutorial_240bus.md",
            "Small-Signal Analysis" => "tutorials/tutorial_continuation_pf.md",
            "Active Load Model" => "tutorials/tutorial_activeload.md",
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
    repo = "github.com/NREL-Sienna/PowerSimulationsDynamics.jl.git",
    target = "build",
    branch = "gh-pages",
    devbranch = "main",
    devurl = "dev",
    push_preview = true,
    versions = ["stable" => "v^", "v#.#"],
)
