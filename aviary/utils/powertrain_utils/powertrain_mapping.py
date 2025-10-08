#!/usr/bin/env python3
"""
Powertrain Mapping File Generator

This module provides utilities for generating powertrain mapping files that describe
the performance characteristics of hybrid-electric propulsion systems across different
operating conditions (Mach number, altitude, throttle settings).

The generated files follow the same format as engine deck files and can be used
directly with Aviary's propulsion system.

Classes
-------
PowertrainMapper : Main class for generating powertrain mapping files

Example
-------
    from aviary.subsystems.propulsion.powertrain_mapping import PowertrainMapper
    
    # Define operating conditions
    mach_numbers = [0.0, 0.3, 0.6, 0.9]
    altitudes = [0.0, 10000.0, 20000.0, 37000.0, 39000.0]  # ft
    throttle_settings = [21.0, 22.0, 24.0, 26.0, 30.0, 34.0, 38.0, 42.0, 46.0, 48.0, 50.0]
    
    # Create mapper
    mapper = PowertrainMapper(
        mach_numbers=mach_numbers,
        altitudes=altitudes,
        throttle_settings=throttle_settings
    )
    
    # Generate mapping file
    mapper.generate_mapping_file(
        output_file='my_powertrain.csv',
        thrust_function=my_thrust_function,
        drag_function=my_drag_function,
        fuel_flow_function=my_fuel_flow_function,
        nox_rate_function=my_nox_function,
        electric_power_function=my_electric_function
    )
"""

import csv
import numpy as np
from datetime import datetime
from typing import Callable, List, Optional, Union
from pathlib import Path


class PowertrainMapper:
    """
    Generator for powertrain mapping files compatible with Aviary engine decks.
    
    This class creates CSV files that map operating conditions (Mach, altitude, throttle)
    to powertrain performance parameters (thrust, drag, fuel flow, NOx rate, electric power).
    
    Parameters
    ----------
    mach_numbers : List[float]
        List of Mach numbers to evaluate
    altitudes : List[float]
        List of altitudes in feet to evaluate
    throttle_settings : List[float]
        List of throttle settings (typically 0-100) to evaluate
    """
    
    def __init__(
        self,
        mach_numbers: List[float],
        altitudes: List[float],
        throttle_settings: List[float],
        supplied_power_ratios: Optional[List[float]] = None
    ):
        self.mach_numbers = sorted(mach_numbers)
        self.altitudes = sorted(altitudes)
        self.throttle_settings = sorted(throttle_settings)
        
        # If supplied_power_ratios not provided, default to [1.0] (single value)
        if supplied_power_ratios is None:
            self.supplied_power_ratios = [1.0]
        else:
            self.supplied_power_ratios = sorted(supplied_power_ratios)
        
        # Calculate total number of data points
        self.total_points = (len(mach_numbers) * len(altitudes) * 
                            len(throttle_settings) * len(self.supplied_power_ratios))
        
    def generate_mapping_file(
        self,
        output_file: Union[str, Path],
        thrust_function: Callable[[float, float, float, float], float],
        drag_function: Callable[[float, float, float, float], float],
        fuel_flow_function: Callable[[float, float, float, float], float],
        nox_rate_function: Callable[[float, float, float, float], float],
        electric_power_function: Callable[[float, float, float, float], float],
        shaft_power_ratio_function: Optional[Callable[[float, float, float, float], float]] = None,
        description: str = "Custom powertrain mapping file",
        author: str = "PowertrainMapper"
    ) -> None:
        """
        Generate a powertrain mapping file.
        
        Parameters
        ----------
        output_file : str or Path
            Path to the output CSV file
        thrust_function : callable
            Function that returns gross thrust (lbf) given (mach, altitude_ft, throttle, supplied_power_ratio)
        drag_function : callable
            Function that returns ram drag (lbf) given (mach, altitude_ft, throttle, supplied_power_ratio)
        fuel_flow_function : callable
            Function that returns fuel flow (lb/h) given (mach, altitude_ft, throttle, supplied_power_ratio)
        nox_rate_function : callable
            Function that returns NOx rate (lb/h) given (mach, altitude_ft, throttle, supplied_power_ratio)
        electric_power_function : callable
            Function that returns electric power (kW) given (mach, altitude_ft, throttle, supplied_power_ratio)
        shaft_power_ratio_function : callable, optional
            Function that returns shaft power ratio (-) given (mach, altitude_ft, throttle, supplied_power_ratio).
            If None, defaults to 1.0 for all conditions.
        description : str
            Description to include in the file header
        author : str
            Author name to include in the file header
        """
        output_path = Path(output_file)
        
        print(f"Generating powertrain mapping file: {output_path}")
        print(f"Total data points: {self.total_points}")
        print(f"Mach numbers: {len(self.mach_numbers)} points")
        print(f"Altitudes: {len(self.altitudes)} points")
        print(f"Throttle settings: {len(self.throttle_settings)} points")
        print(f"Supplied power ratios: {len(self.supplied_power_ratios)} points")
        
        with open(output_path, 'w', newline='') as csvfile:
            # Write header comments
            csvfile.write(f"# created {datetime.now().strftime('%m/%d/%y')}\n")
            csvfile.write(f"# {description}\n")
            csvfile.write(f"# generated by {author} using PowertrainMapper\n")
            csvfile.write(f"# NOTE: this powertrain is for example/testing purposes only\n")
            csvfile.write("\n")
            
            # Write column headers
            csvfile.write("Mach_Number, Altitude (ft), Supplied_Power_Ratio (-), Shaft_Power_Ratio (-),   Throttle, Gross_Thrust (lbf), Ram_Drag (lbf), Fuel_Flow (lb/h), NOx_Rate (lb/h), Electric_Power (kW)\n")
            
            # Generate data points
            point_count = 0
            for mach in self.mach_numbers:
                for altitude in self.altitudes:
                    for supplied_power_ratio in self.supplied_power_ratios:
                        for throttle in self.throttle_settings:
                            # Calculate performance parameters
                            thrust = thrust_function(mach, altitude, throttle, supplied_power_ratio)
                            drag = drag_function(mach, altitude, throttle, supplied_power_ratio)
                            fuel_flow = fuel_flow_function(mach, altitude, throttle, supplied_power_ratio)
                            nox_rate = nox_rate_function(mach, altitude, throttle, supplied_power_ratio)
                            electric_power = electric_power_function(mach, altitude, throttle, supplied_power_ratio)
                            
                            # Calculate shaft power ratio (default to 1.0 if not provided)
                            if shaft_power_ratio_function is not None:
                                shaft_power_ratio = shaft_power_ratio_function(mach, altitude, throttle, supplied_power_ratio)
                            else:
                                shaft_power_ratio = 1.0
                            
                            # Write data row
                            csvfile.write(f"{mach:9.1f}, {altitude:11.1f}, {supplied_power_ratio:23.6f}, {shaft_power_ratio:18.6f}, {throttle:8.1f}, {thrust:15.1f}, {drag:13.1f}, {fuel_flow:13.1f}, {nox_rate:11.4f}, {electric_power:15.3f}\n")
                            
                            point_count += 1
                            if point_count % 100 == 0:
                                print(f"Generated {point_count}/{self.total_points} data points...")
        
        print(f"Successfully generated {output_path} with {point_count} data points")
    
    def get_operating_conditions(self) -> List[tuple]:
        """
        Get all combinations of operating conditions.
        
        Returns
        -------
        List[tuple]
            List of (mach, altitude, supplied_power_ratio, throttle) tuples
        """
        conditions = []
        for mach in self.mach_numbers:
            for altitude in self.altitudes:
                for supplied_power_ratio in self.supplied_power_ratios:
                    for throttle in self.throttle_settings:
                        conditions.append((mach, altitude, supplied_power_ratio, throttle))
        return conditions
    
    def print_summary(self) -> None:
        """Print a summary of the mapping configuration."""
        print("Powertrain Mapping Configuration:")
        print(f"  Mach numbers: {self.mach_numbers}")
        print(f"  Altitudes (ft): {self.altitudes}")
        print(f"  Supplied power ratios: {self.supplied_power_ratios}")
        print(f"  Throttle settings: {self.throttle_settings}")
        print(f"  Total data points: {self.total_points}")


# Example functions for testing
def dummy_thrust_function(mach: float, altitude_ft: float, throttle: float, supplied_power_ratio: float) -> float:
    """Dummy thrust function for testing."""
    base_thrust = 1000.0 + throttle * 500.0
    mach_factor = 1.0 - 0.1 * mach
    altitude_factor = 1.0 - altitude_ft / 100000.0
    return base_thrust * mach_factor * altitude_factor


def dummy_drag_function(mach: float, altitude_ft: float, throttle: float, supplied_power_ratio: float) -> float:
    """Dummy ram drag function for testing."""
    return mach * 100.0 + throttle * 10.0


def dummy_fuel_flow_function(mach: float, altitude_ft: float, throttle: float, supplied_power_ratio: float) -> float:
    """Dummy fuel flow function for testing."""
    base_flow = 100.0 + throttle * 50.0
    mach_factor = 1.0 + 0.2 * mach
    altitude_factor = 1.0 + altitude_ft / 50000.0
    # Fuel flow decreases with higher supplied_power_ratio (more electric power)
    fuel_factor = 1.0 - supplied_power_ratio * 0.5
    return base_flow * mach_factor * altitude_factor * fuel_factor


def dummy_nox_rate_function(mach: float, altitude_ft: float, throttle: float, supplied_power_ratio: float) -> float:
    """Dummy NOx rate function for testing."""
    return (throttle * 0.1 + mach * 0.05) * (1.0 - supplied_power_ratio * 0.5)


def dummy_electric_power_function(mach: float, altitude_ft: float, throttle: float, supplied_power_ratio: float) -> float:
    """Dummy electric power function for testing."""
    base_power = 10.0 + throttle * 5.0
    mach_factor = 1.0 + 0.1 * mach
    altitude_factor = 1.0 - altitude_ft / 200000.0
    # Electric power increases with supplied_power_ratio
    electric_factor = 0.5 + supplied_power_ratio * 0.5
    return base_power * mach_factor * altitude_factor * electric_factor


def generate_dummy_powertrain_file():
    """Generate a dummy powertrain file for testing."""
    # Define operating conditions similar to the reference file
    mach_numbers = [0.0, 0.3, 0.6, 0.9]
    altitudes = [0.0, 2000.0, 10000.0, 20000.0, 30000.0, 37000.0, 39000.0]
    throttle_settings = [21.0, 22.0, 24.0, 26.0, 30.0, 34.0, 38.0, 42.0, 46.0, 48.0, 50.0]
    
    # Create mapper
    mapper = PowertrainMapper(
        mach_numbers=mach_numbers,
        altitudes=altitudes,
        throttle_settings=throttle_settings
    )
    
    # Print configuration summary
    mapper.print_summary()
    
    # Generate the mapping file
    mapper.generate_mapping_file(
        output_file='dummy_powertrain.csv',
        thrust_function=dummy_thrust_function,
        drag_function=dummy_drag_function,
        fuel_flow_function=dummy_fuel_flow_function,
        nox_rate_function=dummy_nox_rate_function,
        electric_power_function=dummy_electric_power_function,
        description="Dummy powertrain mapping file for testing PowertrainMapper",
        author="PowertrainMapper Test"
    )


if __name__ == "__main__":
    # Generate a dummy file for testing
    generate_dummy_powertrain_file()
