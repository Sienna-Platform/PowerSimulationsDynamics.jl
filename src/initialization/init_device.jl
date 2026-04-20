function initialize_dynamic_device!(
    dynamic_device::DynamicWrapper{DynG},
    static::PSY.StaticInjection,
    initial_inner_vars::AbstractVector,
) where {DynG <: PSY.DynamicGenerator}
    #Obtain States
    device_states = zeros(PSY.get_n_states(dynamic_device))

    #Initialize Machine and Shaft: δ and ω
    initialize_mach_shaft!(device_states, static, dynamic_device, initial_inner_vars)
    #Initialize extra Shaft states
    initialize_shaft!(device_states, static, dynamic_device, initial_inner_vars)
    #Initialize AVR
    initialize_avr!(device_states, static, dynamic_device, initial_inner_vars)
    #Initialize TG
    initialize_tg!(device_states, static, dynamic_device, initial_inner_vars)
    #Initialize PSS
    initialize_pss!(device_states, static, dynamic_device, initial_inner_vars)

    return device_states
end

function initialize_dynamic_device!(
    dynamic_device::DynamicWrapper{DynI},
    static::PSY.StaticInjection,
    initial_inner_vars::AbstractVector,
) where {DynI <: PSY.DynamicInverter}
    #Obtain States
    device_states = zeros(PSY.get_n_states(dynamic_device))

    #Initialize Machine and Shaft: V and I
    initialize_filter!(device_states, static, dynamic_device, initial_inner_vars)
    #Initialize freq estimator
    initialize_frequency_estimator!(
        device_states,
        static,
        dynamic_device,
        initial_inner_vars,
    )
    #Initialize OuterLoop
    initialize_outer!(device_states, static, dynamic_device, initial_inner_vars)
    #Initialize DCside
    initialize_DCside!(device_states, static, dynamic_device, initial_inner_vars)
    #Initialize Converter
    initialize_converter!(device_states, static, dynamic_device, initial_inner_vars)
    #Initialize InnerLoop
    initialize_inner!(device_states, static, dynamic_device, initial_inner_vars)
    return device_states
end

function initialize_static_device!(::StaticLoadWrapper)
    return
end

function initialize_static_device!(
    device::StaticWrapper{PSY.Source, T},
) where {T <: BusCategory}
    #PowerFlow Data
    P0 = PSY.get_active_power(device)
    Q0 = PSY.get_reactive_power(device)
    Vm = PSY.get_magnitude(PSY.get_bus(device))
    θ = PSY.get_angle(PSY.get_bus(device))
    S0 = P0 + Q0 * 1im
    V_R = Vm * cos(θ)
    V_I = Vm * sin(θ)
    V = V_R + V_I * 1im
    I = conj(S0 / V)
    I_R = real(I)
    I_I = imag(I)
    R_th = PSY.get_R_th(device.device)
    X_th = PSY.get_X_th(device.device)
    Zmag = R_th^2 + X_th^2

    function f!(out, x)
        V_R_internal = x[1]
        V_I_internal = x[2]

        out[1] =
            R_th * (V_R_internal - V_R) / Zmag + X_th * (V_I_internal - V_I) / Zmag - I_R
        out[2] =
            R_th * (V_I_internal - V_I) / Zmag - X_th * (V_R_internal - V_R) / Zmag - I_I
    end
    x0 = [V_R, V_I]
    sol = NLsolve.nlsolve(f!, x0)
    if !NLsolve.converged(sol)
        @warn("Initialization in Source failed")
    else
        sol_x0 = sol.zero
        #Update terminal voltages
        V_internal = sqrt(sol_x0[1]^2 + sol_x0[2]^2)
        θ_internal = atan(sol_x0[2], sol_x0[1])
        PSY.set_internal_voltage!(device.device, V_internal)
        PSY.set_internal_angle!(device.device, θ_internal)
        set_V_ref(device, PSY.get_internal_voltage(device.device))
        set_θ_ref(device, PSY.get_internal_angle(device.device))
    end
    return
end

function initialize_dynamic_device!(
    dynamic_device::DynamicWrapper{PSY.PeriodicVariableSource},
    source::PSY.Source,
    ::AbstractVector,
)
    device_states = zeros(PSY.get_n_states(dynamic_device))

    #PowerFlow Data
    P0 = PSY.get_active_power(source)
    Q0 = PSY.get_reactive_power(source)
    Vm = PSY.get_magnitude(PSY.get_bus(source))
    θ = PSY.get_angle(PSY.get_bus(source))
    S0 = P0 + Q0 * 1im
    V_R = Vm * cos(θ)
    V_I = Vm * sin(θ)
    V = V_R + V_I * 1im
    I = conj(S0 / V)
    I_R = real(I)
    I_I = imag(I)
    R_th = PSY.get_R_th(source)
    X_th = PSY.get_X_th(source)
    Zmag = R_th^2 + X_th^2
    function f!(out, x)
        V_R_internal = x[1]
        V_I_internal = x[2]

        out[1] =
            R_th * (V_R_internal - V_R) / Zmag + X_th * (V_I_internal - V_I) / Zmag - I_R
        out[2] =
            R_th * (V_I_internal - V_I) / Zmag - X_th * (V_R_internal - V_R) / Zmag - I_I
    end
    x0 = [V_R, V_I]
    sol = NLsolve.nlsolve(f!, x0)
    if !NLsolve.converged(sol)
        @warn("Initialization in Periodic Variable Source failed")
    else
        sol_x0 = sol.zero
        #Update terminal voltages
        V_internal = sqrt(sol_x0[1]^2 + sol_x0[2]^2)
        θ_internal = atan(sol_x0[2], sol_x0[1])

        V_internal_freqs = 0.0
        V_freqs = PSY.get_internal_voltage_frequencies(get_device(dynamic_device))
        V_coeff = PSY.get_internal_voltage_coefficients(get_device(dynamic_device))
        for (ix, ω) in enumerate(V_freqs)
            V_internal_freqs += V_coeff[ix][2]     #sin(0) = 0; cos(0)=1
        end

        θ_internal_freqs = 0.0
        θ_freqs = PSY.get_internal_angle_frequencies(get_device(dynamic_device))
        θ_coeff = PSY.get_internal_angle_coefficients(get_device(dynamic_device))
        for (ix, ω) in enumerate(θ_freqs)
            θ_internal_freqs += θ_coeff[ix][2]     #sin(0) = 0; cos(0)=1
        end
        V_internal_bias = V_internal - V_internal_freqs
        θ_internal_bias = θ_internal - θ_internal_freqs

        device_states[1] = V_internal
        device_states[2] = θ_internal
        PSY.set_internal_voltage_bias!(get_device(dynamic_device), V_internal_bias)
        PSY.set_internal_angle_bias!(get_device(dynamic_device), θ_internal_bias)
    end
    return device_states
end

function initialize_dynamic_device!(branch::BranchWrapper)
    device_states = zeros(PSY.get_n_states(branch))
    #PowerFlow Data
    arc = PSY.get_arc(branch)
    Vm_from = PSY.get_magnitude(PSY.get_from(arc))
    θ_from = PSY.get_angle(PSY.get_from(arc))
    Vm_to = PSY.get_magnitude(PSY.get_to(arc))
    θ_to = PSY.get_angle(PSY.get_to(arc))
    V_R_from = Vm_from * cos(θ_from)
    V_I_from = Vm_from * sin(θ_from)
    V_R_to = Vm_to * cos(θ_to)
    V_I_to = Vm_to * sin(θ_to)
    R = PSY.get_r(branch)
    X = PSY.get_x(branch)
    Zmag_sq = R^2 + X^2
    #Compute Current
    I_R = R * (V_R_from - V_R_to) / Zmag_sq + X * (V_I_from - V_I_to) / Zmag_sq
    I_I = R * (V_I_from - V_I_to) / Zmag_sq - X * (V_R_from - V_R_to) / Zmag_sq
    #Update Current
    device_states[1] = I_R
    device_states[2] = I_I
    return device_states
end

function initialize_dynamic_device!(
    dynamic_wrapper::DynamicWrapper{PSY.SingleCageInductionMachine},
    device::PSY.StaticInjection,
    ::AbstractVector,
)
    Sbase = get_system_base_power(dynamic_wrapper)

    #Obtain States
    device_states = zeros(PSY.get_n_states(dynamic_wrapper))

    # Get parameters
    dynamic_device = get_device(dynamic_wrapper)
    R_s = PSY.get_R_s(dynamic_device)
    X_ls = PSY.get_X_ls(dynamic_device)
    R_r = PSY.get_R_r(dynamic_device)
    X_lr = PSY.get_X_lr(dynamic_device)
    A = PSY.get_A(dynamic_device)
    B = PSY.get_B(dynamic_device)
    C = PSY.get_C(dynamic_device)
    base_power = PSY.get_base_power(dynamic_device)
    X_ad = PSY.get_X_ad(dynamic_device)
    X_aq = PSY.get_X_aq(dynamic_device)

    #PowerFlow Data
    if isa(device, PSY.StandardLoad)
        P0 = get_total_p(device) * Sbase / base_power # in pu (motor base)
        Q0 = get_total_q(device) * Sbase / base_power # in pu (motor base)
    else
        P0 = PSY.get_active_power(device) * Sbase / base_power # in pu (motor base)
        Q0 = PSY.get_reactive_power(device) * Sbase / base_power # in pu (motor base)
    end
    Vm = PSY.get_magnitude(PSY.get_bus(device))
    θ = PSY.get_angle(PSY.get_bus(device))
    S0 = P0 + Q0 * 1im
    V_R = Vm * cos(θ)
    V_I = Vm * sin(θ)
    V = V_R + V_I * 1im
    I = conj(S0 / V) # total current (includes B_shunt + motor)
    I_R = real(I)
    I_I = imag(I)

    # voltages in qd reference frame
    v_ds = V_R
    v_qs = V_I

    # Initial guess for NLSolve (assume B_shunt = 0)
    i_qs0 = I_I
    i_ds0 = I_R
    B_sh0 = 0.0
    ψ_qs0 = -v_ds + R_s * i_ds0
    ψ_ds0 = v_qs - R_s * i_qs0
    ψ_mq0 = ψ_qs0 - i_qs0 * X_ls
    ψ_md0 = ψ_ds0 - i_ds0 * X_ls
    ψ_qr0 = (ψ_mq0 / X_ad - ψ_qs0 / X_ls) * X_lr
    ψ_dr0 = (ψ_md0 / X_ad - ψ_ds0 / X_ls) * X_lr
    ωr0 = 0.98 # good guess for ind. motor
    τ_m00 = P0 / ωr0
    x0 = [i_qs0, i_ds0, B_sh0, ψ_qs0, ψ_ds0, ψ_qr0, ψ_dr0, ωr0, τ_m00]

    #
    function f!(out, x)
        i_qs = x[1]
        i_ds = x[2]
        B_sh = x[3]
        ψ_qs = x[4]
        ψ_ds = x[5]
        ψ_qr = x[6]
        ψ_dr = x[7]
        ωr = x[8]
        τ_m0 = x[9]
        ψ_mq = ψ_qs - i_qs * X_ls
        ψ_md = ψ_ds - i_ds * X_ls
        out[1] = -I_R + i_ds - V_I * B_sh # network interface
        out[2] = -I_I + i_qs + V_R * B_sh # network interface
        out[3] = v_qs - ψ_ds - R_s * i_qs #
        out[4] = v_ds + ψ_qs - R_s * i_ds #
        out[5] = -ψ_mq + X_aq * (ψ_qs / X_ls + ψ_qr / X_lr) # dψ_qs/dt = 0
        out[6] = -ψ_md + X_ad * (ψ_ds / X_ls + ψ_dr / X_lr) # dψ_ds/dt = 0
        out[7] = -(1.0 - ωr) * ψ_dr + R_r / X_lr * (ψ_mq - ψ_qr) # dψ_qr/dt = 0
        out[8] = (1.0 - ωr) * ψ_qr + R_r / X_lr * (ψ_md - ψ_dr) # dψ_dr/dt = 0
        out[9] = ψ_ds * i_qs - ψ_qs * i_ds - τ_m0 * (A * ωr^2 + B * ωr + C) # dωr/dt = 0
    end
    sol = NLsolve.nlsolve(f!, x0; ftol = STRICT_NLSOLVE_F_TOLERANCE)
    if !NLsolve.converged(sol)
        @warn("Initialization in Ind. Motor $(PSY.get_name(device)) failed")
    else
        sol_x0 = sol.zero
        device_states[1] = sol_x0[4] # ψ_qs
        device_states[2] = sol_x0[5] # ψ_ds
        device_states[3] = sol_x0[6] # ψ_qr
        device_states[4] = sol_x0[7] # ψ_dr
        device_states[5] = sol_x0[8] # ωr
        # update τ_ref and B_sh
        PSY.set_B_shunt!(dynamic_device, sol_x0[3]) # B_sh
        #set_B_shunt(dynamic_device, sol_x0[3]) # B_sh
        PSY.set_τ_ref!(dynamic_device, sol_x0[9]) # τ_m0
        set_P_ref(dynamic_wrapper, sol_x0[9]) # τ_m0
    end
    return device_states
end

function initialize_dynamic_device!(
    dynamic_wrapper::DynamicWrapper{PSY.SimplifiedSingleCageInductionMachine},
    device::PSY.StaticInjection,
    ::AbstractVector,
)
    Sbase = get_system_base_power(dynamic_wrapper)

    #Obtain States
    device_states = zeros(PSY.get_n_states(dynamic_wrapper))

    # Get parameters
    dynamic_device = get_device(dynamic_wrapper)
    R_s = PSY.get_R_s(dynamic_device)
    X_m = PSY.get_X_m(dynamic_device)
    R_r = PSY.get_R_r(dynamic_device)
    A = PSY.get_A(dynamic_device)
    B = PSY.get_B(dynamic_device)
    C = PSY.get_C(dynamic_device)
    base_power = PSY.get_base_power(dynamic_device)
    X_ss = PSY.get_X_ss(dynamic_device)
    X_rr = PSY.get_X_rr(dynamic_device)
    X_p = PSY.get_X_p(dynamic_device)

    #PowerFlow Data
    if isa(device, PSY.StandardLoad)
        P0 = get_total_p(device) * Sbase / base_power # in pu (motor base)
        Q0 = get_total_q(device) * Sbase / base_power # in pu (motor base)
    else
        P0 = PSY.get_active_power(device) * Sbase / base_power # in pu (motor base)
        Q0 = PSY.get_reactive_power(device) * Sbase / base_power # in pu (motor base)
    end
    Vm = PSY.get_magnitude(PSY.get_bus(device))
    θ = PSY.get_angle(PSY.get_bus(device))
    S0 = P0 + Q0 * 1im
    V_R = Vm * cos(θ)
    V_I = Vm * sin(θ)
    V = V_R + V_I * 1im
    I = conj(S0 / V) # total current (includes B_shunt + motor)
    I_R = real(I)
    I_I = imag(I)

    # voltages in qd reference frame
    v_ds = V_R
    v_qs = V_I

    # Initial guess for NLSolve (assume B_shunt = 0)
    i_qs0 = I_I
    i_ds0 = I_R
    B_sh0 = 0.0
    ψ_qs0 = -v_ds + R_s * i_ds0
    ψ_ds0 = v_qs - R_s * i_qs0
    ψ_qr0 = X_rr / X_m * (ψ_qs0 - i_qs0 * X_p)
    ψ_dr0 = X_rr / X_m * (ψ_ds0 - i_ds0 * X_p)
    i_qr0 = (ψ_qr0 - X_m * i_qs0) / X_rr
    i_dr0 = (ψ_dr0 - X_m * i_ds0) / X_rr
    ωr0 = 0.98 # good guess for ind. motor
    τ_m00 = P0 / ωr0
    x0 = [i_qs0, i_ds0, i_qr0, i_dr0, B_sh0, ψ_qs0, ψ_ds0, ψ_qr0, ψ_dr0, ωr0, τ_m00]

    #
    function f!(out, x)
        i_qs = x[1]
        i_ds = x[2]
        i_qr = x[3]
        i_dr = x[4]
        B_sh = x[5]
        ψ_qs = x[6]
        ψ_ds = x[7]
        ψ_qr = x[8]
        ψ_dr = x[9]
        ωr = x[10]
        τ_m0 = x[11]

        out[1] = -I_R + i_ds - V_I * B_sh # network interface
        out[2] = -I_I + i_qs + V_R * B_sh # network interface
        out[3] = v_qs - ψ_ds - R_s * i_qs # dψ_qs/dt = 0
        out[4] = v_ds + ψ_qs - R_s * i_ds # dψ_ds/dt = 0
        out[5] = -ψ_qs + X_ss * i_qs + X_m * i_qr
        out[6] = -ψ_ds + X_ss * i_ds + X_m * i_dr
        out[7] = -ψ_qr + X_rr * i_qr + X_m * i_qs
        out[8] = -ψ_dr + X_rr * i_dr + X_m * i_ds
        out[9] = -(1.0 - ωr) * ψ_dr - R_r * i_qr # dψ_qr/dt = 0
        out[10] = (1.0 - ωr) * ψ_qr - R_r * i_dr # dψ_dr/dt = 0
        out[11] = ψ_qr * i_dr - ψ_dr * i_qr - τ_m0 * (A * ωr^2 + B * ωr + C) # dωr/dt = 0
    end
    sol = NLsolve.nlsolve(f!, x0; ftol = STRICT_NLSOLVE_F_TOLERANCE)
    if !NLsolve.converged(sol)
        @warn("Initialization in Ind. Motor $(PSY.get_name(device)) failed")
    else
        sol_x0 = sol.zero
        device_states[1] = sol_x0[8] # ψ_qr
        device_states[2] = sol_x0[9] # ψ_dr
        device_states[3] = sol_x0[10] # ωr
        # update τ_ref and B_sh
        PSY.set_B_shunt!(dynamic_device, sol_x0[5]) # B_sh
        #set_B_shunt(dynamic_device, sol_x0[5]) # B_sh
        PSY.set_τ_ref!(dynamic_device, sol_x0[11]) # τ_m0
        set_P_ref(dynamic_wrapper, sol_x0[11]) # τ_m0
    end
    return device_states
end

function initialize_dynamic_device!(
    dynamic_wrapper::DynamicWrapper{PSY.CSVGN1},
    device::PSY.StaticInjection,
    ::AbstractVector,
)
    Sbase = get_system_base_power(dynamic_wrapper)

    # Obtain States
    device_states = zeros(PSY.get_n_states(dynamic_wrapper))

    # Get parameters
    dynamic_device = get_device(dynamic_wrapper)
    K = PSY.get_K(dynamic_device)
    T1 = PSY.get_T1(dynamic_device)
    T2 = PSY.get_T2(dynamic_device)
    T3 = PSY.get_T3(dynamic_device)
    T4 = PSY.get_T4(dynamic_device)
    T5 = PSY.get_T5(dynamic_device)
    Rmin = PSY.get_Rmin(dynamic_device)
    Vmax = PSY.get_Vmax(dynamic_device)
    Vmin = PSY.get_Vmin(dynamic_device)
    Cbase = PSY.get_CBase(dynamic_device)
    # FIXME: base_power is changed to system's base_power when a CSVGN1 is attached to a Source using add_component!()
    # Temporarily, to avoid that, set_dynamic_injector!() could be used
    Mbase = PSY.get_base_power(dynamic_device)
    Rbase = Mbase

    # PowerFlow Data
    Q0 = PSY.get_reactive_power(device) # in pu (system base)
    # TODO: V_abs is the voltage magnitude on the high side of generator step-up transformer, if present.
    V_abs = PSY.get_magnitude(PSY.get_bus(device))
    Y = Q0 / V_abs^2

    V_ref0 = V_abs - (Cbase / Sbase - Y) * 1 / K * Sbase / Mbase

    # update V_ref
    set_V_ref(dynamic_wrapper, V_ref0)

    thy = K * (V_abs - V_ref0)
    vr2 = thy

    if (thy > Vmax) || (thy < Vmin)
        @error("Thyristor state thy = $(thy) outside the limits")
    end

    if (vr2 > 1) || (vr2 < Rmin / Rbase)
        @error("Regulator state vr2 = $(vr2) outside the limits")
    end

    device_states[1] = K * (V_abs - V_ref0) # thy
    device_states[2] = 0.0 # vr1
    device_states[3] = K * (V_abs - V_ref0) # vr2

    return device_states
end

function initialize_dynamic_device!(
    dynamic_wrapper::DynamicWrapper{PSY.ActiveConstantPowerLoad},
    device::PSY.StaticInjection,
    ::AbstractVector,
)
    Sbase = get_system_base_power(dynamic_wrapper)

    #Obtain States
    device_states = zeros(PSY.get_n_states(dynamic_wrapper))

    # Get parameters
    dynamic_device = get_device(dynamic_wrapper)
    r_load = PSY.get_r_load(dynamic_device)
    rf = PSY.get_rf(dynamic_device)
    lf = PSY.get_lf(dynamic_device)
    cf = PSY.get_cf(dynamic_device)
    rg = PSY.get_rg(dynamic_device)
    lg = PSY.get_lg(dynamic_device)
    kiv = PSY.get_kiv(dynamic_device)
    kic = PSY.get_kic(dynamic_device)
    base_power = PSY.get_base_power(dynamic_device)

    #PowerFlow Data
    if isa(device, PSY.StandardLoad)
        P0 = get_total_p(device) * Sbase / base_power # in pu (motor base)
        Q0 = get_total_q(device) * Sbase / base_power # in pu (motor base)
    else
        P0 = PSY.get_active_power(device) * Sbase / base_power # in pu (motor base)
        Q0 = PSY.get_reactive_power(device) * Sbase / base_power # in pu (motor base)
    end
    Vm = PSY.get_magnitude(PSY.get_bus(device))
    θ = PSY.get_angle(PSY.get_bus(device))
    S0 = P0 + Q0 * 1im
    V_R = Vm * cos(θ)
    V_I = Vm * sin(θ)
    V = V_R + V_I * 1im
    I = conj(S0 / V) # total current
    I_R = real(I)
    I_I = imag(I)

    V_filter0 = V - I * (rg + lg * 1im)
    Vr_filter0 = real(V_filter0)
    Vi_filter0 = imag(V_filter0)
    θ_pll0 = angle(V_filter0)

    Ir_cap = -cf * Vi_filter0
    Ii_cap = +cf * Vr_filter0
    I_cnv0 = I - (Ir_cap + 1im * Ii_cap)
    Ir_cnv0 = real(I_cnv0)
    Ii_cnv0 = imag(I_cnv0)
    V_cnv0 = V_filter0 - I_cnv0 * (rf + lf * 1im)
    Vr_cnv0 = real(V_cnv0)
    Vi_cnv0 = imag(V_cnv0)
    P_cnv0 = Vr_cnv0 * Ir_cnv0 + Vi_cnv0 * Ii_cnv0
    V_ref0 = sqrt(P_cnv0 * r_load)
    x0 = [θ_pll0, 0.0, 0.0, 0.0, 0.0]

    function f!(out, x)
        θ_pll = x[1]
        η = x[2]
        γd = x[3]
        γq = x[4]
        Iq_ref = x[5]

        V_dq_pll = ri_dq(θ_pll + pi / 2) * [Vr_filter0; Vi_filter0]
        I_dq_cnv = ri_dq(θ_pll + pi / 2) * [Ir_cnv0; Ii_cnv0]

        V_dq_cnv0 = ri_dq(θ_pll + pi / 2) * [Vr_cnv0; Vi_cnv0]

        Id_ref = kiv * η

        Vd_cnv = kic * γd + 1.0 * lf * I_dq_cnv[q]
        Vq_cnv = kic * γq - 1.0 * lf * I_dq_cnv[d]

        out[1] = V_dq_pll[q]
        out[2] = Id_ref - I_dq_cnv[d]
        out[3] = Iq_ref - I_dq_cnv[q]
        out[4] = V_dq_cnv0[q] - Vq_cnv
        out[5] = V_dq_cnv0[d] - Vd_cnv
    end
    sol = NLsolve.nlsolve(f!, x0; ftol = STRICT_NLSOLVE_F_TOLERANCE)
    if !NLsolve.converged(sol)
        @warn("Initialization in Active Load $(PSY.get_name(device)) failed")
    else
        sol_x0 = sol.zero
        device_states[1] = sol_x0[1] # θ_pll
        device_states[2] = 0.0 # ϵ
        device_states[3] = sol_x0[2] # η
        device_states[4] = V_ref0
        device_states[5] = sol_x0[3] # γd
        device_states[6] = sol_x0[4] # γq
        device_states[7] = Ir_cnv0
        device_states[8] = Ii_cnv0
        device_states[9] = Vr_filter0
        device_states[10] = Vi_filter0
        device_states[11] = I_R
        device_states[12] = I_I

        # update V_ref
        PSY.set_V_ref!(dynamic_device, V_ref0)
        set_V_ref(dynamic_wrapper, V_ref0)
        set_Q_ref(dynamic_wrapper, sol_x0[5])
    end
    return device_states
end

function initialize_dynamic_device!(
    dynamic_wrapper::DynamicWrapper{PSY.AggregateDistributedGenerationA},
    static::PSY.StaticInjection,
    initial_inner_vars::AbstractVector,
)
    device_states = zeros(PSY.get_n_states(dynamic_wrapper))
    dynamic_device = get_device(dynamic_wrapper)

    #Get PowerFlow Data
    P0 = PSY.get_active_power(static)
    Q0 = PSY.get_reactive_power(static)
    Vm = PSY.get_magnitude(PSY.get_bus(static))
    θ = PSY.get_angle(PSY.get_bus(static))
    S0 = P0 + Q0 * 1im

    V_R = Vm * cos(θ)
    V_I = Vm * sin(θ)
    V = V_R + V_I * 1im
    I = conj(S0 / V)

    Ip = real(I * exp(-im * θ))
    Iq_neg = imag(I * exp(-im * θ))
    Iq = -Iq_neg

    Vmeas = Vm
    Fmeas = 1.0
    Freq_ref = 1.0
    Mult = 1.0
    Ip_cmd = Ip / Mult
    Iq_cmd = Iq / Mult
    Ip_min, Ip_max, Iq_min, Iq_max = current_limit_logic(dynamic_device, Ip_cmd, Iq_cmd)
    if Ip_cmd >= Ip_max + BOUNDS_TOLERANCE || Ip_min - BOUNDS_TOLERANCE >= Ip_cmd
        @error(
            "Inverter $(PSY.get_name(static)) active current $(Ip_cmd) out of limits $(Ip_min) $(Ip_max). Check Power Flow or Parameters"
        )
    end

    if Iq_cmd >= Iq_max + BOUNDS_TOLERANCE || Iq_min - BOUNDS_TOLERANCE >= Iq_cmd
        @error(
            "Inverter $(PSY.get_name(static)) reactive current $(Iq_cmd) out of limits $(Iq_min) $(Iq_max). Check Power Flow or Parameters"
        )
    end

    Pord = Ip_cmd * max(Vmeas, 0.01)
    dPord = Pord
    Pmeas = Pord
    Q_V = Iq_cmd
    pfaref = atan(Q0 / P0)
    Qref = Iq_cmd * max(Vmeas, 0.01)
    Freq_Flag = PSY.get_Freq_Flag(dynamic_device)
    if Freq_Flag == 0
        Pref = dPord
        device_states[1] = Vmeas
        device_states[2] = Pmeas
        device_states[3] = Q_V
        device_states[4] = Iq
        device_states[5] = Mult
        device_states[6] = Fmeas
        device_states[7] = Ip
    elseif Freq_Flag == 1
        Pref = Pmeas
        PowerPI = dPord
        device_states[1] = Vmeas
        device_states[2] = Pmeas
        device_states[3] = Q_V
        device_states[4] = Iq
        device_states[5] = Mult
        device_states[6] = Fmeas
        device_states[7] = PowerPI
        device_states[8] = dPord
        device_states[9] = Pord
        device_states[10] = Ip
    else
        @error "Unsupported value of Freq_Flag"
    end

    #See Note 2 on PSSE Documentation
    Vref0 = PSY.get_V_ref(dynamic_device)
    K_qv = PSY.get_K_qv(dynamic_device)
    (dbd1, dbd2) = PSY.get_dbd_pnts(dynamic_device)
    if Vref0 == 0.0
        Vref = Vmeas
    elseif dbd1 <= (Vref0 - Vmeas) * K_qv <= dbd2
        Vref = Vref0
    else
        Vref = Vmeas
    end

    set_P_ref(dynamic_wrapper, Pref)
    PSY.set_P_ref!(dynamic_device, Pref)
    set_Q_ref(dynamic_wrapper, Qref)
    set_V_ref(dynamic_wrapper, Vref)
    set_ω_ref(dynamic_wrapper, Freq_ref)
    PSY.set_Pfa_ref!(dynamic_device, pfaref)

    return device_states
end
