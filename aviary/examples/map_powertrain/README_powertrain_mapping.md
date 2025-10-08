# Powertrain Mapping Framework

This framework provides utilities for generating powertrain mapping files that describe the performance characteristics of hybrid-electric propulsion systems across different operating conditions using the `power_transmission_computation` function from `component_matices.py`.

## Overview

The `PowertrainMapper` class generates CSV files that map operating conditions (Mach number, altitude, throttle settings, supplied power ratio) to powertrain performance parameters (thrust, drag, fuel flow, NOx rate, electric power). These files are compatible with Aviary's engine deck system.

## Files

### Core Framework
- `../../utils/powertrain_utils/powertrain_mapping.py` - Main framework with the `PowertrainMapper` class
- `../../utils/powertrain_utils/component_matices.py` - Power transmission computation functions

### Examples and Tests
- `example_powertrain_generator.py` - Example script for generating serial hybrid powertrain mapping files
- `../../utils/powertrain_utils/test/test_serial_hybrid_powertrain.py` - Test suite for validating computations
- `README_powertrain_mapping.md` - This documentation file

## Quick Start

### Running the Example

```bash
cd aviary/examples/map_powertrain
python example_powertrain_generator.py
```

This will:
1. Display the powertrain configuration
2. Run the power transmission computation
3. Generate `serial_hybrid_powertrain.csv` with 1540 data points (4 Mach × 7 altitudes × 5 supplied power ratios × 11 throttle settings)

### Running the Tests

```bash
cd aviary
python -m aviary.utils.powertrain_utils.test.test_serial_hybrid_powertrain
```

This will validate:
1. Power transmission computation for various operating conditions
2. Thrust calculation from propulsive power
3. Energy balance across the powertrain

## Using the Framework

### Basic Usage with Supplied Power Ratio

```python
from aviary.utils.powertrain_utils.powertrain_mapping import PowertrainMapper

# Define operating conditions
mach_numbers = [0.0, 0.3, 0.6, 0.9]
altitudes = [0.0, 10000.0, 20000.0, 37000.0]  # ft
supplied_power_ratios = [0.0, 0.25, 0.5, 0.75, 1.0]  # 0=all fuel, 1=all electric
throttle_settings = [21.0, 30.0, 40.0, 50.0]

# Create mapper
mapper = PowertrainMapper(
    mach_numbers=mach_numbers,
    altitudes=altitudes,
    throttle_settings=throttle_settings,
    supplied_power_ratios=supplied_power_ratios  # Optional, defaults to [1.0]
)

# Define your performance functions (must accept 4 parameters)
def my_thrust_function(mach, altitude_ft, throttle, supplied_power_ratio):
    # Your thrust calculation logic here
    return thrust_value

def my_drag_function(mach, altitude_ft, throttle, supplied_power_ratio):
    # Your ram drag calculation logic here
    return drag_value

def my_fuel_flow_function(mach, altitude_ft, throttle, supplied_power_ratio):
    # Your fuel flow calculation logic here
    return fuel_flow_value

def my_nox_rate_function(mach, altitude_ft, throttle, supplied_power_ratio):
    # Your NOx rate calculation logic here
    return nox_rate_value

def my_electric_power_function(mach, altitude_ft, throttle, supplied_power_ratio):
    # Your electric power calculation logic here
    return electric_power_value

# Generate the mapping file
mapper.generate_mapping_file(
    output_file='my_powertrain.csv',
    thrust_function=my_thrust_function,
    drag_function=my_drag_function,
    fuel_flow_function=my_fuel_flow_function,
    nox_rate_function=my_nox_rate_function,
    electric_power_function=my_electric_power_function,
    description="My custom powertrain mapping file",
    author="My Name"
)
```

### Using PowertrainConfig Class

For cleaner code organization, use the `PowertrainConfig` dataclass to store configuration parameters:

```python
from dataclasses import dataclass
from typing import Dict

@dataclass
class PowertrainConfig:
    """Configuration class for powertrain parameters."""
    config: str  # 'serial', 'parallel', 'turboelectric', etc.
    etas: Dict[str, float]  # Component efficiencies
    P_inst: float  # Installed power in Watts

# Define configuration once
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

# Use in your functions
def calculate_thrust(mach, altitude_ft, throttle, supplied_power_ratio, 
                    config=SERIAL_HYBRID_CONFIG):
    # Function implementation uses config.etas, config.P_inst, etc.
    pass
```

## Performance Function Requirements

Each performance function must accept **four parameters**:
- `mach` (float): Mach number
- `altitude_ft` (float): Altitude in feet
- `throttle` (float): Throttle setting (typically 0-100)
- `supplied_power_ratio` (float): Supplied power ratio (0-1), where 0 = all fuel, 1 = all electric

And return the appropriate performance parameter:

### Required Functions

1. **thrust_function** → Returns gross thrust in lbf
2. **drag_function** → Returns ram drag in lbf
3. **fuel_flow_function** → Returns fuel flow in lb/h
4. **nox_rate_function** → Returns NOx rate in lb/h
5. **electric_power_function** → Returns electric power in kW

### Optional Functions

6. **shaft_power_ratio_function** → Returns shaft power ratio (-), defaults to 1.0 if not provided

## Output File Format

The generated CSV files include the supplied power ratio column:

```csv
# created MM/DD/YY
# Description of the powertrain mapping file
# generated by Author using PowertrainMapper
# NOTE: this powertrain is for example/testing purposes only

Mach_Number, Altitude (ft), Supplied_Power_Ratio (-), Shaft_Power_Ratio (-),   Throttle, Gross_Thrust (lbf), Ram_Drag (lbf), Fuel_Flow (lb/h), NOx_Rate (lb/h), Electric_Power (kW)
      0.0,         0.0,                0.000000,           1.000000,     21.0,           919.9,           0.0,         213.0,      3.1954,          -0.000
      0.0,         0.0,                0.250000,           1.000000,     21.0,          1340.6,           0.0,         213.0,      3.1954,         128.333
      0.0,         0.0,                0.500000,           1.000000,     21.0,          2181.9,           0.0,         213.0,      3.1954,         385.000
      ...
```

## Integration with Aviary

The generated powertrain mapping files can be used directly with Aviary's engine deck system by specifying the file path in your aircraft configuration:

```python
# In your aircraft configuration file
aircraft:engine:data_file,path/to/your/powertrain.csv,unitless
```

## Power Transmission Computation

The framework uses `power_transmission_computation` from `component_matices.py` to calculate power flow through various hybrid-electric powertrain configurations.

### Supported Configurations

- **conventional**: Traditional gas turbine only
- **turboelectric**: GT → EM → Motor → Propulsor
- **serial**: GT → Generator → Battery ⇄ Motor → Propulsor
- **parallel**: GT + Motor → Propulsor
- **PTE** (Partial Turboelectric): GT → (Propulsor + EM → Motor → Propulsor)
- **SPPH** (Series-Parallel Partial Hybrid): Full hybrid with all components
- **e-1**: Pure electric (primary motor)
- **e-2**: Pure electric (secondary motor)
- **dual-e**: Dual electric motors

### Configuration Parameters

For each configuration, you need to specify:

1. **config** (str): Powertrain type (see above)
2. **etas** (dict): Component efficiencies
   - GT: Gas turbine efficiency
   - GB: Gearbox efficiency
   - P1/P2: Propulsor 1/2 efficiency
   - EM1/EM2: Electric motor/generator 1/2 efficiency
   - PM: Power management efficiency
3. **supplied_power_ratio** (float): P_bat / (P_bat + P_fuel), range 0-1
4. **shaft_power_ratio** (float): P_s2 / (P_s2 + P_s1) for configurations with two shafts
5. **throttle_setting** (float): Normalized throttle (0-1)
6. **P_inst** (float): Maximum installed power [W] of GT or EM

### Degrees of Freedom (DOF)

Different configurations require different numbers of specified inputs:

- **Conventional, Turboelectric, e-1, e-2**: 1 DOF
- **Serial, Parallel, PTE, Dual-e**: 2 DOFs
- **SPPH**: 3 DOFs

The framework automatically determines which parameters need to be specified.

## Advanced Usage

### Custom Configurations

To use a different powertrain configuration, modify the `PowertrainConfig`:

```python
# Parallel hybrid configuration
PARALLEL_HYBRID_CONFIG = PowertrainConfig(
    config='parallel',
    etas={'GT': 0.92, 'GB': 0.95, 'P1': 0.85, 'EM1': 0.95, 'PM': 0.98, 'EM2': 0.95, 'P2': 0.85},
    P_inst=2.0e6  # 2 MW
)
```

### Varying Supplied Power Ratio

The supplied power ratio allows you to explore the design space of hybrid-electric aircraft:

```python
# Range from conventional (0.0) to pure electric (1.0)
supplied_power_ratios = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
```

This generates data for:
- 0.0: All power from fuel (conventional)
- 0.5: Equal contribution from fuel and battery
- 1.0: All power from battery (pure electric)

### Atmospheric Calculations

The framework includes a simplified standard atmosphere model:

```python
def calculate_speed_of_sound(altitude_ft):
    """Calculate speed of sound using ISA model."""
    altitude_m = altitude_ft * 0.3048
    
    if altitude_m <= 11000:  # Troposphere
        T = 288.15 - 0.0065 * altitude_m
    else:  # Lower stratosphere
        T = 216.65
    
    return np.sqrt(1.4 * 287.05 * T)  # m/s
```

### Thrust from Propulsive Power

The conversion from propulsive power to thrust:

```python
# Thrust = Propulsive Power / Flight Speed
thrust_N = P_propulsive / flight_speed_ms
thrust_lbf = thrust_N * 0.224809
```

For static conditions (Mach = 0), use a nominal reference speed (e.g., 50 m/s) to avoid division by zero.

## Validation and Testing

### Test Suite

Run the comprehensive test suite:

```bash
python -m aviary.utils.powertrain_utils.test.test_serial_hybrid_powertrain
```

Tests include:
1. **Power transmission computation**: Validates power flow for 8 different operating conditions
2. **Thrust calculation**: Verifies thrust computation from propulsive power at 3 flight conditions
3. **Energy balance**: Checks energy conservation through the powertrain

### Expected Results

For serial hybrid at 50% supplied power ratio and 50% throttle:
- Fuel power: ~917 kW
- Battery power: ~917 kW
- Propulsive power: ~1155 kW
- Overall efficiency: ~0.63 (due to multiple component losses)

## Tips for Creating Realistic Performance Functions

1. **Thrust Functions**: 
   - Account for compressibility effects at high Mach
   - Include air density effects at altitude
   - Use propulsive power from `power_transmission_computation`

2. **Fuel Flow Functions**:
   - Derive from GT power output (P_out['gt'])
   - Use realistic thermal efficiency (~30% for gas turbines)
   - Account for fuel energy content (~18,500 Btu/lb)

3. **Electric Power Functions**:
   - Use battery power from P_out['bat']
   - Positive values indicate discharge, negative indicates charging
   - Convert to kW for output

4. **NOx Functions**:
   - Scale with fuel flow
   - Use emissions index (typical: 15 g NOx / kg fuel)

5. **Configuration Consistency**:
   - Define config once at module level
   - Pass config object to all functions
   - Ensures consistency across all performance calculations

## Troubleshooting

### Common Issues

1. **"For a 'serial' configuration, exactly 2 DOF(s) must be specified"**
   - Solution: Provide both `supplied_power_ratio` and `throttle_setting`

2. **Division by zero in thrust calculation**
   - Solution: Use minimum flight speed for static/low-speed conditions

3. **Negative electric power**
   - Expected: Indicates battery charging (regenerative operation)
   - For discharge-only: Check supplied_power_ratio logic

4. **Unrealistic thrust values at Mach = 0**
   - Solution: Use reference speed (e.g., 50 m/s) instead of actual flight speed

### Performance Optimization

For large datasets:
- Use vectorized NumPy operations
- Consider parallel processing for independent function calls
- Monitor memory usage with large operating condition matrices

## Contributing

To extend the framework:
1. Add new powertrain configurations in `component_matices.py`
2. Create example scripts for new configurations
3. Add test cases for validation
4. Update documentation with new features

## References

- Aviary Documentation: https://github.com/OpenMDAO/Aviary
- Standard Atmosphere: ISA (International Standard Atmosphere)
- Component Matices: Based on hybrid-electric aircraft powertrain modeling

## License

This framework is part of the Aviary project and follows the same licensing terms.
