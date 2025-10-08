#!/usr/bin/env python3
"""
Example: Serial Hybrid Powertrain Mapping Generator

This example demonstrates how to generate powertrain mapping files using the
power_transmission_computation function from component_matices.py.

The example shows how to:
1. Configure a serial hybrid powertrain
2. Define operating conditions (Mach, altitude, throttle, supplied_power_ratio)
3. Generate a complete powertrain mapping file compatible with Aviary

Author: PowertrainMapper with component_matices
Date: 2025-10-08
"""

import numpy as np
from dataclasses import dataclass
from typing import Dict
from aviary.utils.powertrain_utils.powertrain_mapping import PowertrainMapper
from aviary.utils.powertrain_utils.component_matices import power_transmission_computation


@dataclass
class PowertrainConfig:
    """
    Configuration class for powertrain parameters.
    
    Attributes
    ----------
    config : str
        Powertrain configuration type ('serial', 'parallel', 'turboelectric', etc.)
    etas : dict
        Component efficiencies
    P_inst : float
        Installed power in Watts
    """
    config: str
    etas: Dict[str, float]
    P_inst: float


# Global configuration for the serial hybrid powertrain
SERIAL_HYBRID_CONFIG = PowertrainConfig(
    config='serial',
    etas={
        'GT': 0.90,
        'GB': 0.90,
        'P1': 0.90,
        'EM1': 0.90,
        'PM': 0.90,
        'EM2': 0.90,
        'P2': 0.90
    },
    P_inst=1.65e6  # 1.65 MW
)


def calculate_speed_of_sound(altitude_ft: float) -> float:
    """
    Calculate speed of sound using standard atmosphere model.
    
    Parameters
    ----------
    altitude_ft : float
        Altitude in feet
        
    Returns
    -------
    float
        Speed of sound in m/s
    """
    # Convert altitude to meters
    altitude_m = altitude_ft * 0.3048
    
    # Standard atmosphere model (simplified)
    if altitude_m <= 11000:  # Troposphere
        T = 288.15 - 0.0065 * altitude_m  # K
    else:  # Lower stratosphere
        T = 216.65  # K
    
    # Speed of sound: a = sqrt(gamma * R * T)
    # gamma = 1.4 for air, R = 287.05 J/(kg·K)
    speed_of_sound = np.sqrt(1.4 * 287.05 * T)
    
    return speed_of_sound


def calculate_thrust_from_powertrain(
    mach: float, 
    altitude_ft: float, 
    throttle: float,
    supplied_power_ratio: float,
    config: PowertrainConfig = SERIAL_HYBRID_CONFIG
) -> float:
    """
    Calculate thrust using power_transmission_computation function.
    
    Parameters
    ----------
    mach : float
        Mach number
    altitude_ft : float
        Altitude in feet
    throttle : float
        Throttle setting (0-100)
    supplied_power_ratio : float
        Supplied power ratio (0-1): P_bat / (P_bat + P_fuel)
    config : PowertrainConfig
        Powertrain configuration object
        
    Returns
    -------
    float
        Thrust in lbf
    """
    # Normalize throttle to 0-1
    throttle_setting = throttle / 100.0
    
    # Call power_transmission_computation
    P_out, xi_out, phi_out, Phi_out, solution, throttle_config = power_transmission_computation(
        config=config.config,
        etas=config.etas,
        supplied_power_ratio=supplied_power_ratio,
        shaft_power_ratio=np.nan,  # For serial, this is set internally to 1.0
        throttle_setting=throttle_setting,
        P_p=np.nan,
        P_p1=np.nan,
        P_p2=np.nan,
        P_inst=config.P_inst
    )
    
    # Calculate flight speed: V = M * a (speed of sound)
    speed_of_sound_ms = calculate_speed_of_sound(altitude_ft)
    flight_speed_ms = mach * speed_of_sound_ms
    
    # For static (Mach = 0) conditions, use a nominal reference speed
    if flight_speed_ms < 1.0:  # Less than 1 m/s (~2 knots)
        flight_speed_ms = 50.0  # Use 50 m/s (~100 knots) for thrust calculation
    
    # Propulsive power to thrust: Thrust = P_propulsive / V
    # P_out['p'] is in Watts, flight_speed is in m/s
    thrust_N = P_out['p'] / flight_speed_ms
    thrust_lbf = thrust_N * 0.224809  # N to lbf
    
    return thrust_lbf


def calculate_ram_drag(
    mach: float,
    altitude_ft: float,
    throttle: float,
    supplied_power_ratio: float,
    config: PowertrainConfig = SERIAL_HYBRID_CONFIG
) -> float:
    """
    Calculate ram drag for the powertrain.
    
    Parameters
    ----------
    mach : float
        Mach number
    altitude_ft : float
        Altitude in feet
    throttle : float
        Throttle setting (0-100)
    supplied_power_ratio : float
        Supplied power ratio (0-1)
    config : PowertrainConfig
        Powertrain configuration object
        
    Returns
    -------
    float
        Ram drag in lbf
    """
    # Ram drag scales with Mach squared
    mach_factor = mach**2
    
    # Altitude effects: lower air density reduces ram drag
    altitude_factor = (1.0 - altitude_ft / 100000.0)
    
    # Base ram drag coefficient
    base_ram_drag = 2000.0  # lbf at M=1.0, sea level
    
    return base_ram_drag * mach_factor * altitude_factor


def calculate_fuel_flow_from_powertrain(
    mach: float,
    altitude_ft: float,
    throttle: float,
    supplied_power_ratio: float,
    config: PowertrainConfig = SERIAL_HYBRID_CONFIG
) -> float:
    """
    Calculate fuel flow using power_transmission_computation function.
    
    Parameters
    ----------
    mach : float
        Mach number
    altitude_ft : float
        Altitude in feet
    throttle : float
        Throttle setting (0-100)
    supplied_power_ratio : float
        Supplied power ratio (0-1)
    config : PowertrainConfig
        Powertrain configuration object
        
    Returns
    -------
    float
        Fuel flow in lb/h
    """
    throttle_setting = throttle / 100.0
    
    # Call power_transmission_computation
    P_out, xi_out, phi_out, Phi_out, solution, throttle_config = power_transmission_computation(
        config=config.config,
        etas=config.etas,
        supplied_power_ratio=supplied_power_ratio,
        shaft_power_ratio=np.nan,
        throttle_setting=throttle_setting,
        P_p=np.nan,
        P_p1=np.nan,
        P_p2=np.nan,
        P_inst=config.P_inst
    )
    
    # GT power is P_out['gt'] in Watts
    # Convert to fuel flow using thermal efficiency
    # Thermal efficiency of ~30% for gas turbines
    # Energy content of jet fuel: ~18,500 Btu/lb
    
    P_gt_W = P_out['gt']
    
    thermal_efficiency = 0.30
    fuel_energy_content_Wh_per_lb = 18500.0 * 1055.06 / 3600.0  # Btu/lb to Wh/lb
    
    # Fuel flow = Power / (efficiency * energy_content)
    if P_gt_W > 0:
        fuel_flow_lb_h = P_gt_W / (thermal_efficiency * fuel_energy_content_Wh_per_lb)
    else:
        fuel_flow_lb_h = 0.0
    
    return fuel_flow_lb_h


def calculate_nox_rate_from_fuel_flow(
    mach: float,
    altitude_ft: float,
    throttle: float,
    supplied_power_ratio: float,
    config: PowertrainConfig = SERIAL_HYBRID_CONFIG
) -> float:
    """
    Calculate NOx rate based on fuel flow.
    
    Parameters
    ----------
    mach : float
        Mach number
    altitude_ft : float
        Altitude in feet
    throttle : float
        Throttle setting (0-100)
    supplied_power_ratio : float
        Supplied power ratio (0-1)
    config : PowertrainConfig
        Powertrain configuration object
        
    Returns
    -------
    float
        NOx rate in lb/h
    """
    # Get fuel flow
    fuel_flow = calculate_fuel_flow_from_powertrain(mach, altitude_ft, throttle, supplied_power_ratio, config)
    
    # NOx emissions index: typical value is 15 g NOx / kg fuel = 0.015 lb NOx / lb fuel
    nox_emissions_index = 0.015
    
    nox_rate = fuel_flow * nox_emissions_index
    
    return nox_rate


def calculate_electric_power_from_powertrain(
    mach: float,
    altitude_ft: float,
    throttle: float,
    supplied_power_ratio: float,
    config: PowertrainConfig = SERIAL_HYBRID_CONFIG
) -> float:
    """
    Calculate electric power using power_transmission_computation function.
    
    Parameters
    ----------
    mach : float
        Mach number
    altitude_ft : float
        Altitude in feet
    throttle : float
        Throttle setting (0-100)
    supplied_power_ratio : float
        Supplied power ratio (0-1)
    config : PowertrainConfig
        Powertrain configuration object
        
    Returns
    -------
    float
        Electric power in kW
    """
    throttle_setting = throttle / 100.0
    
    # Call power_transmission_computation
    P_out, xi_out, phi_out, Phi_out, solution, throttle_config = power_transmission_computation(
        config=config.config,
        etas=config.etas,
        supplied_power_ratio=supplied_power_ratio,
        shaft_power_ratio=np.nan,
        throttle_setting=throttle_setting,
        P_p=np.nan,
        P_p1=np.nan,
        P_p2=np.nan,
        P_inst=config.P_inst
    )
    
    # Battery power is P_out['bat'] in Watts
    # Convert to kW
    electric_power_kW = P_out['bat'] / 1000.0
    
    return electric_power_kW


def calculate_shaft_power_ratio(
    mach: float,
    altitude_ft: float,
    throttle: float,
    supplied_power_ratio: float,
    config: PowertrainConfig = SERIAL_HYBRID_CONFIG
) -> float:
    """
    Calculate shaft power ratio for serial configuration.
    
    For serial configuration, shaft_power_ratio = 1.0 (all power goes to secondary shaft).
    
    Parameters
    ----------
    mach : float
        Mach number
    altitude_ft : float
        Altitude in feet
    throttle : float
        Throttle setting (0-100)
    supplied_power_ratio : float
        Supplied power ratio (0-1)
    config : PowertrainConfig
        Powertrain configuration object
        
    Returns
    -------
    float
        Shaft power ratio (dimensionless)
    """
    return 1.0


def generate_serial_hybrid_powertrain():
    """Generate a serial hybrid powertrain mapping file using power_transmission_computation."""
    
    # Define operating conditions
    mach_numbers = [0.0, 0.3, 0.6, 0.9]
    altitudes = [0.0, 2000.0, 10000.0, 20000.0, 30000.0, 37000.0, 39000.0]  # ft
    supplied_power_ratios = [0.0, 0.25, 0.5, 0.75, 1.0]  # Range from all fuel to all electric
    throttle_settings = [21.0, 22.0, 24.0, 26.0, 30.0, 34.0, 38.0, 42.0, 46.0, 48.0, 50.0]
    
    # Create mapper
    mapper = PowertrainMapper(
        mach_numbers=mach_numbers,
        altitudes=altitudes,
        throttle_settings=throttle_settings,
        supplied_power_ratios=supplied_power_ratios
    )
    
    # Print configuration summary
    print("=" * 80)
    print("Serial Hybrid Powertrain Mapping Generator")
    print("=" * 80)
    print(f"\nPowertrain Configuration:")
    print(f"  Type: {SERIAL_HYBRID_CONFIG.config}")
    print(f"  Installed Power: {SERIAL_HYBRID_CONFIG.P_inst/1e6:.2f} MW")
    print(f"  Component Efficiencies: {SERIAL_HYBRID_CONFIG.etas['GT']}")
    print()
    mapper.print_summary()
    print()
    
    # Generate the mapping file
    mapper.generate_mapping_file(
        output_file='serial_hybrid_powertrain.csv',
        thrust_function=calculate_thrust_from_powertrain,
        drag_function=calculate_ram_drag,
        fuel_flow_function=calculate_fuel_flow_from_powertrain,
        nox_rate_function=calculate_nox_rate_from_fuel_flow,
        electric_power_function=calculate_electric_power_from_powertrain,
        shaft_power_ratio_function=calculate_shaft_power_ratio,
        description="Serial hybrid powertrain mapping using power_transmission_computation",
        author="PowertrainMapper with component_matices"
    )
    
    print(f"\n{'=' * 80}")
    print("SUCCESS: Serial hybrid powertrain file generated!")
    print(f"{'=' * 80}")
    print(f"\nOutput file: serial_hybrid_powertrain.csv")
    print(f"This file can now be used with Aviary's engine deck system.\n")


if __name__ == "__main__":
    generate_serial_hybrid_powertrain()
