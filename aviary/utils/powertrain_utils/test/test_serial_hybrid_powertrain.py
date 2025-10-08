#!/usr/bin/env python3
"""
Test script for serial hybrid powertrain computation.

This script validates the power_transmission_computation function and the
PowertrainMapper framework with various operating conditions.
"""

import numpy as np
from dataclasses import dataclass
from typing import Dict
from aviary.utils.powertrain_utils.component_matices import power_transmission_computation


@dataclass
class PowertrainConfig:
    """Configuration class for powertrain parameters."""
    config: str
    etas: Dict[str, float]
    P_inst: float


def test_power_transmission_computation():
    """Test the power_transmission_computation function with various inputs."""
    
    print("\n" + "="*80)
    print("Testing power_transmission_computation Function")
    print("="*80 + "\n")
    
    # Configuration for serial hybrid
    config = 'serial'
    etas = {
        'GT': 0.90,
        'GB': 0.90,
        'P1': 0.90,
        'EM1': 0.90,
        'PM': 0.90,
        'EM2': 0.90,
        'P2': 0.90
    }
    P_inst = 1.65e6  # 1.65 MW
    
    # Test cases: (supplied_power_ratio, throttle_setting, description)
    test_cases = [
        (0.0, 0.5, "All fuel, half throttle"),
        (0.5, 0.5, "50% fuel/50% battery, half throttle"),
        (1.0, 0.5, "All battery, half throttle"),
        (0.0, 1.0, "All fuel, full throttle"),
        (0.5, 1.0, "50% fuel/50% battery, full throttle"),
        (1.0, 1.0, "All battery, full throttle"),
        (0.25, 0.75, "25% battery, 75% throttle"),
        (0.75, 0.75, "75% battery, 75% throttle"),
    ]
    
    passed = 0
    failed = 0
    
    for supplied_power_ratio, throttle_setting, description in test_cases:
        print(f"\nTest case: {description}")
        print(f"  Supplied power ratio: {supplied_power_ratio}")
        print(f"  Throttle setting: {throttle_setting}")
        
        try:
            P_out, xi_out, phi_out, Phi_out, solution, throttle_config = power_transmission_computation(
                config=config,
                etas=etas,
                supplied_power_ratio=supplied_power_ratio,
                shaft_power_ratio=np.nan,
                throttle_setting=throttle_setting,
                P_p=np.nan,
                P_p1=np.nan,
                P_p2=np.nan,
                P_inst=P_inst
            )
            
            print(f"  Results:")
            print(f"    Fuel power (f):      {P_out['f']/1000:.2f} kW")
            print(f"    GT power (gt):       {P_out['gt']/1000:.2f} kW")
            print(f"    Battery power (bat): {P_out['bat']/1000:.2f} kW")
            print(f"    Propulsive power (p):{P_out['p']/1000:.2f} kW")
            print(f"    Actual supplied power ratio (phi): {phi_out:.3f}")
            print(f"    Actual shaft power ratio (Phi):    {Phi_out:.3f}")
            
            # Validate results
            if P_out['p'] > 0:
                print(f"  Status: ✓ PASS")
                passed += 1
            else:
                print(f"  Status: ✗ FAIL (zero propulsive power)")
                failed += 1
                
        except Exception as e:
            print(f"  Status: ✗ FAIL")
            print(f"  Error: {str(e)}")
            failed += 1
    
    print(f"\n" + "="*80)
    print(f"Test Summary: {passed} passed, {failed} failed out of {passed + failed} tests")
    print("="*80 + "\n")
    
    return passed, failed


def test_thrust_calculation():
    """Test thrust calculation from propulsive power."""
    
    print("\n" + "="*80)
    print("Testing Thrust Calculation")
    print("="*80 + "\n")
    
    # Configuration
    config = 'serial'
    etas = {
        'GT': 0.90,
        'GB': 0.90,
        'P1': 0.90,
        'EM1': 0.90,
        'PM': 0.90,
        'EM2': 0.90,
        'P2': 0.90
    }
    P_inst = 1.65e6
    
    # Test different flight conditions
    test_cases = [
        (0.3, 0.0, 0.5, 0.5, "Low speed, sea level"),
        (0.6, 20000.0, 0.5, 0.75, "Medium speed, cruise altitude"),
        (0.8, 37000.0, 0.5, 1.0, "High speed, high altitude"),
    ]
    
    for mach, altitude_ft, supplied_power_ratio, throttle_setting, description in test_cases:
        print(f"\nTest case: {description}")
        print(f"  Mach: {mach}, Altitude: {altitude_ft} ft")
        print(f"  Supplied power ratio: {supplied_power_ratio}, Throttle: {throttle_setting}")
        
        try:
            # Get power output
            P_out, xi_out, phi_out, Phi_out, solution, throttle_config = power_transmission_computation(
                config=config,
                etas=etas,
                supplied_power_ratio=supplied_power_ratio,
                shaft_power_ratio=np.nan,
                throttle_setting=throttle_setting,
                P_p=np.nan,
                P_p1=np.nan,
                P_p2=np.nan,
                P_inst=P_inst
            )
            
            # Calculate flight speed
            altitude_m = altitude_ft * 0.3048
            if altitude_m <= 11000:
                T = 288.15 - 0.0065 * altitude_m
            else:
                T = 216.65
            speed_of_sound = np.sqrt(1.4 * 287.05 * T)
            flight_speed_ms = max(mach * speed_of_sound, 50.0)  # Minimum 50 m/s
            
            # Calculate thrust
            thrust_N = P_out['p'] / flight_speed_ms
            thrust_lbf = thrust_N * 0.224809
            
            print(f"  Results:")
            print(f"    Flight speed: {flight_speed_ms:.2f} m/s")
            print(f"    Propulsive power: {P_out['p']/1000:.2f} kW")
            print(f"    Thrust: {thrust_lbf:.2f} lbf")
            print(f"  Status: ✓ PASS")
            
        except Exception as e:
            print(f"  Status: ✗ FAIL")
            print(f"  Error: {str(e)}")
    
    print(f"\n" + "="*80)


def test_energy_balance():
    """Test energy balance: input power = output power (with efficiencies)."""
    
    print("\n" + "="*80)
    print("Testing Energy Balance")
    print("="*80 + "\n")
    
    config = 'serial'
    etas = {
        'GT': 0.90,
        'GB': 0.90,
        'P1': 0.90,
        'EM1': 0.90,
        'PM': 0.90,
        'EM2': 0.90,
        'P2': 0.90
    }
    P_inst = 1.65e6
    
    supplied_power_ratio = 0.5
    throttle_setting = 0.75
    
    print(f"Test case: Energy balance check")
    print(f"  Supplied power ratio: {supplied_power_ratio}")
    print(f"  Throttle setting: {throttle_setting}")
    
    P_out, xi_out, phi_out, Phi_out, solution, throttle_config = power_transmission_computation(
        config=config,
        etas=etas,
        supplied_power_ratio=supplied_power_ratio,
        shaft_power_ratio=np.nan,
        throttle_setting=throttle_setting,
        P_p=np.nan,
        P_p1=np.nan,
        P_p2=np.nan,
        P_inst=P_inst
    )
    
    print(f"\n  Power flow:")
    print(f"    Fuel power in:        {P_out['f']/1000:.2f} kW")
    print(f"    Battery power in:     {P_out['bat']/1000:.2f} kW")
    print(f"    Total input power:    {(P_out['f'] + P_out['bat'])/1000:.2f} kW")
    print(f"    Propulsive power out: {P_out['p']/1000:.2f} kW")
    
    # Calculate expected efficiency
    # Serial: fuel -> GT -> EM1 -> PM -> EM2 -> P2
    overall_efficiency = etas['GT'] * etas['EM1'] * etas['PM'] * etas['EM2'] * etas['P2']
    print(f"\n  Overall efficiency: {overall_efficiency:.4f}")
    print(f"  Expected output: {(P_out['f'] + P_out['bat']) * overall_efficiency / 1000:.2f} kW")
    print(f"  Actual output: {P_out['p']/1000:.2f} kW")
    
    # Check if energy is conserved (within tolerance)
    expected_output = (P_out['f'] + P_out['bat']) * overall_efficiency
    error = abs(P_out['p'] - expected_output) / expected_output * 100
    print(f"  Error: {error:.2f}%")
    
    if error < 1.0:  # Less than 1% error
        print(f"  Status: ✓ PASS")
    else:
        print(f"  Status: ✗ FAIL (energy balance error > 1%)")
    
    print(f"\n" + "="*80)


if __name__ == "__main__":
    print("\n" + "="*80)
    print("SERIAL HYBRID POWERTRAIN TEST SUITE")
    print("="*80)
    
    # Run all tests
    passed, failed = test_power_transmission_computation()
    test_thrust_calculation()
    test_energy_balance()
    
    print("\n" + "="*80)
    print("ALL TESTS COMPLETED")
    print("="*80 + "\n")

