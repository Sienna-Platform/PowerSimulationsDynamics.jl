# The implementation of caches is influenced by the SparseDiffTools.jl. We implement this
# custom code since the structure of the system models is not compatible with the functionalities
# in SparseDiffTools

struct JacobianFunctionWrapper{
    D <: DelayModel,
    F,
    T <: Union{Matrix{Float64}, SparseArrays.SparseMatrixCSC{Float64, Int64}},
} <: Function
    Jf::F
    Jv::T
    x::Vector{Float64}
    mass_matrix::LinearAlgebra.Diagonal{Float64}
end

# This version of the function is type unstable should only be used for non-critial ops
function (J::JacobianFunctionWrapper{NoDelays})(x::AbstractVector{Float64})
    J.x .= x
    return J.Jf(J.Jv, x)
end

function (J::JacobianFunctionWrapper{NoDelays})(
    JM::U,
    x::AbstractVector{Float64},
) where {U <: Union{Matrix{Float64}, SparseArrays.SparseMatrixCSC{Float64, Int64}}}
    J.x .= x
    J.Jf(JM, x)
    return
end

function (J::JacobianFunctionWrapper{HasDelays})(
    JM::U,
    x::AbstractVector{Float64},
) where {U <: Union{Matrix{Float64}, SparseArrays.SparseMatrixCSC{Float64, Int64}}}
    h(p, t; idxs = nothing) = typeof(idxs) <: Number ? x[idxs] : x
    J.x .= x
    J.Jf(JM, x, h, 0.0)
    return
end

function (J::JacobianFunctionWrapper{NoDelays})(
    JM::U,
    x::AbstractVector{Float64},
    p,
    t,
) where {U <: Union{Matrix{Float64}, SparseArrays.SparseMatrixCSC{Float64, Int64}}}
    J(JM, x)
    return
end

function (J::JacobianFunctionWrapper{HasDelays})(
    JM::U,
    x::AbstractVector{Float64},
    h,
    p,
    t,
) where {U <: Union{Matrix{Float64}, SparseArrays.SparseMatrixCSC{Float64, Int64}}}
    J.Jf(JM, x, h, t)
    return
end

function (J::JacobianFunctionWrapper{NoDelays})(
    JM::U,
    x::AbstractVector{Float64},
    h,
    p,
    t,
) where {U <: Union{Matrix{Float64}, SparseArrays.SparseMatrixCSC{Float64, Int64}}}
    J.Jf(JM, x, h, t)
    return
end

function (J::JacobianFunctionWrapper{NoDelays})(
    JM::U,
    dx::AbstractVector{Float64},
    x::AbstractVector{Float64},
    p,
    gamma::Float64,
    t,
) where {U <: Union{Matrix{Float64}, SparseArrays.SparseMatrixCSC{Float64, Int64}}}
    J(JM, x)
    for i in 1:length(x)
        JM[i, i] -= gamma * J.mass_matrix[i, i]
    end
    return
end

function JacobianFunctionWrapper(
    m!::SystemModel{MassMatrixModel, NoDelays},
    x0_guess::Vector{Float64};
    # Improve the heuristic to do sparsity detection
    sparse_retrieve_loop::Int = 0, #max(3, length(x0_guess) ÷ 100),
)
    x0 = deepcopy(x0_guess)
    n = length(x0)
    m_ = (residual, x) -> m!(residual, x, nothing, 0.0)
    jconfig = ForwardDiff.JacobianConfig(m_, similar(x0), x0, ForwardDiff.Chunk(x0))
    Jf = (Jv, x) -> begin
        @debug "Evaluating Jacobian Function"
        ForwardDiff.jacobian!(Jv, m_, zeros(n), x, jconfig)
        return
    end
    jac = zeros(n, n)
    if sparse_retrieve_loop > 0
        for _ in 1:sparse_retrieve_loop
            temp = zeros(n, n)
            rng_state = copy(Random.default_rng())
            Jf(temp, x0 + Random.randn(n))
            copy!(Random.default_rng(), rng_state)
            jac .+= abs.(temp)
        end
        Jv = SparseArrays.sparse(jac)
    elseif sparse_retrieve_loop == 0
        Jv = jac
    else
        throw(IS.ConflictingInputsError("negative sparse_retrieve_loop not valid"))
    end
    Jf(Jv, x0)
    mass_matrix = get_mass_matrix(m!.inputs)
    return JacobianFunctionWrapper{NoDelays, typeof(Jf), typeof(Jv)}(
        Jf,
        Jv,
        x0,
        mass_matrix,
    )
end

function JacobianFunctionWrapper(
    m!::SystemModel{ResidualModel},
    x0::Vector{Float64};
    # Improve the heuristic to do sparsity detection
    sparse_retrieve_loop::Int = max(3, length(x0) ÷ 100),
)
    n = length(x0)
    m_ = (residual, x) -> m!(residual, zeros(n), x, nothing, 0.0)
    jconfig = ForwardDiff.JacobianConfig(m_, similar(x0), x0, ForwardDiff.Chunk(x0))
    Jf = (Jv, x) -> begin
        @debug "Evaluating Jacobian Function"
        ForwardDiff.jacobian!(Jv, m_, zeros(n), x, jconfig)
        return
    end
    mass_matrix = get_mass_matrix(m!.inputs)
    jac = zeros(n, n)
    if sparse_retrieve_loop > 0
        for _ in 1:sparse_retrieve_loop
            temp = zeros(n, n)
            rng_state = copy(Random.default_rng())
            Jf(temp, x0 + Random.randn(n))
            copy!(Random.default_rng(), rng_state)
            jac .+= abs.(temp)
        end
        Jv = SparseArrays.sparse(jac .+ mass_matrix)
    elseif sparse_retrieve_loop == 0
        Jv = jac
    else
        throw(IS.ConflictingInputsError("negative sparse_retrieve_loop not valid"))
    end
    Jf(Jv, x0)
    mass_matrix = get_mass_matrix(m!.inputs)
    return JacobianFunctionWrapper{NoDelays, typeof(Jf), typeof(Jv)}(
        Jf,
        Jv,
        x0,
        mass_matrix,
    )
end

function JacobianFunctionWrapper(
    m!::SystemModel{MassMatrixModel, HasDelays},
    x0_guess::Vector{Float64};
    # Improve the heuristic to do sparsity detection
    sparse_retrieve_loop::Int = 0, #max(3, length(x0_guess) ÷ 100),
)
    x0 = deepcopy(x0_guess)
    n = length(x0)
    Jf =
        (Jv, x, h, t) -> begin
            @debug "Evaluating Jacobian Function"
            m_ = (residual, x) -> m!(residual, x, h, nothing, t)
            jconfig =
                ForwardDiff.JacobianConfig(m_, similar(x0), x0, ForwardDiff.Chunk(x0))
            ForwardDiff.jacobian!(Jv, m_, zeros(n), x, jconfig)
            return
        end
    jac = zeros(n, n)
    if sparse_retrieve_loop > 0
        h(p, t; idxs = nothing) = typeof(idxs) <: Number ? x0[idxs] : x0
        for _ in 1:sparse_retrieve_loop
            temp = zeros(n, n)
            rng_state = copy(Random.default_rng())
            Jf(temp, x0 + Random.randn(n), h, 0.0)
            copy!(Random.default_rng(), rng_state)
            jac .+= abs.(temp)
        end
        Jv = SparseArrays.sparse(jac)
    elseif sparse_retrieve_loop == 0
        Jv = jac
    else
        throw(IS.ConflictingInputsError("negative sparse_retrieve_loop not valid"))
    end
    mass_matrix = get_mass_matrix(m!.inputs)
    return JacobianFunctionWrapper{HasDelays, typeof(Jf), typeof(Jv)}(
        Jf,
        Jv,
        x0,
        mass_matrix,
    )
end

function get_jacobian(
    ::Type{T},
    inputs::SimulationInputs,
    x0_init::Vector{Float64},
    sparse_retrieve_loop::Int,
) where {T <: SimulationModel}
    return JacobianFunctionWrapper(
        T(inputs, x0_init, JacobianCache),
        x0_init;
        sparse_retrieve_loop = sparse_retrieve_loop,
    )
end

"""
    function get_jacobian(
    ::Type{T},
    system::[`PowerSystems.System`](@extref),
    sparse_retrieve_loop::Int = 3,
    ) where {T <: SimulationModel}

Returns the jacobian function of the system model resulting from the system data.

# Arguments:
- `::SimulationModel` : Type of Simulation Model. `ResidualModel` or `MassMatrixModel`. See [Models Section](https://sienna-platform.github.io/PowerSimulationsDynamics.jl/stable/models/) for more details
- `system`: [`PowerSystems.System`](@extref) — power system data
- `sparse_retrieve_loop::Int` : Number of loops for sparsity detection. If 0, builds the Jacobian with a DenseMatrix
"""
function get_jacobian(
    ::Type{T},
    system::PSY.System,
    sparse_retrieve_loop::Int = 3,
) where {T <: SimulationModel}
    # Deepcopy avoid system modifications
    simulation_system = deepcopy(system)
    inputs = SimulationInputs(T, simulation_system, ReferenceBus())
    x0_init = get_flat_start(inputs)
    set_operating_point!(x0_init, inputs, system)
    return get_jacobian(T, inputs, x0_init, sparse_retrieve_loop)
end
