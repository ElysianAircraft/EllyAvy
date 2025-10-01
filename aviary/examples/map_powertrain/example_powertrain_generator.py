#!/usr/bin/env python3
"""
Example script demonstrating how to use the PowertrainMapper framework.

This script shows how to create custom powertrain mapping files with user-defined
performance functions.
"""

from aviary.utils.powertrain_utils.powertrain_mapping import PowertrainMapper


def realistic_thrust_function(mach: float, altitude_ft: float, throttle: float) -> float:
    """
    Example realistic thrust function for a hybrid turbofan.
    
    This function simulates thrust characteristics that vary with:
    - Throttle setting (primary driver)
    - Mach number (compressibility effects)
    - Altitude (air density effects)
    """
    # Base thrust scales with throttle (0-100 -> 0-1)
    throttle_factor = throttle / 100.0
    
    # Mach effects: thrust decreases at high Mach due to compressibility
    mach_factor = 1.0 - 0.3 * mach**2
    
    # Altitude effects: thrust decreases with altitude due to lower air density
    # Standard atmosphere approximation
    altitude_factor = (1.0 - altitude_ft / 100000.0)**0.5
    
    # Base thrust at sea level, static conditions
    base_thrust = 28000.0  # lbf
    
    return base_thrust * throttle_factor * mach_factor * altitude_factor


def realistic_ram_drag_function(mach: float, altitude_ft: float, throttle: float) -> float:
    """
    Example ram drag function.
    
    Ram drag increases with Mach number and is relatively independent of throttle.
    """
    # Ram drag scales with Mach squared
    mach_factor = mach**2
    
    # Altitude effects: lower air density reduces ram drag
    altitude_factor = (1.0 - altitude_ft / 100000.0)
    
    # Base ram drag coefficient
    base_ram_drag = 2000.0  # lbf at M=1.0, sea level
    
    return base_ram_drag * mach_factor * altitude_factor


def realistic_fuel_flow_function(mach: float, altitude_ft: float, throttle: float) -> float:
    """
    Example fuel flow function.
    
    Fuel flow increases with throttle and has some Mach/altitude dependencies.
    """
    # Primary dependency on throttle
    throttle_factor = (throttle / 100.0)**1.5
    
    # Mach effects: fuel flow increases at high Mach
    mach_factor = 1.0 + 0.2 * mach
    
    # Altitude effects: fuel flow increases with altitude
    altitude_factor = 1.0 + altitude_ft / 50000.0
    
    # Base fuel flow at maximum throttle, sea level, static
    base_fuel_flow = 8000.0  # lb/h
    
    return base_fuel_flow * throttle_factor * mach_factor * altitude_factor


def realistic_nox_rate_function(mach: float, altitude_ft: float, throttle: float) -> float:
    """
    Example NOx rate function.
    
    NOx emissions increase with throttle and have altitude dependencies.
    """
    # Primary dependency on throttle
    throttle_factor = (throttle / 100.0)**2.0
    
    # Altitude effects: NOx increases with altitude
    altitude_factor = 1.0 + altitude_ft / 40000.0
    
    # Base NOx rate at maximum throttle, sea level
    base_nox_rate = 60.0  # lb/h
    
    return base_nox_rate * throttle_factor * altitude_factor


def realistic_electric_power_function(mach: float, altitude_ft: float, throttle: float) -> float:
    """
    Example electric power function for hybrid system.
    
    Electric power varies with operating conditions and throttle.
    """
    # Electric power increases with throttle
    throttle_factor = throttle / 100.0
    
    # Mach effects: electric power may increase at high Mach for cooling/auxiliary systems
    mach_factor = 1.0 + 0.1 * mach
    
    # Altitude effects: electric power may increase with altitude for environmental systems
    altitude_factor = 1.0 + altitude_ft / 100000.0
    
    # Base electric power at maximum throttle, sea level
    base_electric_power = 300.0  # kW
    
    return base_electric_power * throttle_factor * mach_factor * altitude_factor / 10


def realistic_shaft_power_ratio_function(mach: float, altitude_ft: float, throttle: float) -> float:
    """
    Example shaft power ratio function.
    
    For this example, we set it to 1.0 for all conditions.
    In a real application, this could vary based on operating conditions.
    """
    return 1.0


def generate_realistic_powertrain():
    """Generate a realistic powertrain mapping file."""
    
    # Define operating conditions similar to the reference turbofan file
    mach_numbers = [0.0, 0.3, 0.6, 0.9]
    altitudes = [0.0, 2000.0, 10000.0, 20000.0, 30000.0, 37000.0, 39000.0]  # ft
    throttle_settings = [21.0, 22.0, 24.0, 26.0, 30.0, 34.0, 38.0, 42.0, 46.0, 48.0, 50.0]
    
    # Create mapper
    mapper = PowertrainMapper(
        mach_numbers=mach_numbers,
        altitudes=altitudes,
        throttle_settings=throttle_settings
    )
    
    # Print configuration summary
    print("Generating realistic hybrid turbofan powertrain mapping...")
    mapper.print_summary()
    
    # Generate the mapping file
    mapper.generate_mapping_file(
        output_file='realistic_hybrid_turbofan.csv',
        thrust_function=realistic_thrust_function,
        drag_function=realistic_ram_drag_function,
        fuel_flow_function=realistic_fuel_flow_function,
        nox_rate_function=realistic_nox_rate_function,
        electric_power_function=realistic_electric_power_function,
        shaft_power_ratio_function=realistic_shaft_power_ratio_function,
        description="Realistic hybrid turbofan powertrain mapping file",
        author="PowertrainMapper Example"
    )


def generate_custom_powertrain():
    """Generate a custom powertrain with different operating conditions."""
    
    # Custom operating conditions for a different aircraft
    mach_numbers = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]
    altitudes = [0.0, 5000.0, 15000.0, 25000.0, 35000.0, 45000.0]  # ft
    throttle_settings = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0]
    
    # Create mapper
    mapper = PowertrainMapper(
        mach_numbers=mach_numbers,
        altitudes=altitudes,
        throttle_settings=throttle_settings
    )
    
    # Print configuration summary
    print("\nGenerating custom high-speed powertrain mapping...")
    mapper.print_summary()
    
    # Generate the mapping file
    mapper.generate_mapping_file(
        output_file='custom_high_speed_powertrain.csv',
        thrust_function=realistic_thrust_function,
        drag_function=realistic_ram_drag_function,
        fuel_flow_function=realistic_fuel_flow_function,
        nox_rate_function=realistic_nox_rate_function,
        electric_power_function=realistic_electric_power_function,
        shaft_power_ratio_function=realistic_shaft_power_ratio_function,
        description="Custom high-speed powertrain mapping file",
        author="PowertrainMapper Custom Example"
    )


if __name__ == "__main__":
    # Generate realistic powertrain
    generate_realistic_powertrain()
    
    # Generate custom powertrain
    generate_custom_powertrain()
    
    print("\nExample powertrain files generated successfully!")
    print("Files created:")
    print("  - realistic_hybrid_turbofan.csv")
    print("  - custom_high_speed_powertrain.csv")
    print("\nYou can now use these files with Aviary's engine deck system.")
