#!/usr/bin/env python3
"""
Debug script to run check_totals and identify derivative calculation issues.
"""

import aviary.api as av
from aviary.api import Aircraft, Mission, Settings, ProblemType
from aviary.subsystems.energy.battery_builder import BatteryBuilder
from aviary.examples.external_subsystems.custom_aero.custom_aero_builder import CustomAeroBuilder

# Phase info with batteries - copy from working example
phase_info = {
    'pre_mission': {
                'include_takeoff': False,
                'external_subsystems': [BatteryBuilder()],
                'optimize_mass': True,
            },
    'climb': {
        'external_subsystems': [
            CustomAeroBuilder(),
            BatteryBuilder()
            ],
        'subsystem_options': {'core_aerodynamics': {'method': 'external'}},
        'user_options': {
            'num_segments': 5,
            'order': 3,
            'mach_optimize': False,
            'mach_initial': (0.2, 'unitless'),
            'mach_final': (0.5, 'unitless'),
            'mach_bounds': ((0.18, 0.55), 'unitless'),
            'mach_polynomial_order': 3,
            'altitude_optimize': False,
            'altitude_initial': (0.0, 'ft'),
            'altitude_final': (28000.0, 'ft'),
            'altitude_bounds': ((0.0, 34000.0), 'ft'),
            'altitude_polynomial_order': 3,
            'throttle_enforcement': 'path_constraint',
            'time_initial_bounds': ((0.0, 0.0), 'min'),
            'time_duration_bounds': ((30.0, 180.0), 'min'),
        },
    },
    'cruise': {
        'external_subsystems': [
            CustomAeroBuilder(),
            BatteryBuilder()
            ],
        'subsystem_options': {'core_aerodynamics': {'method': 'external'}},
        'user_options': {
            'num_segments': 5,
            'order': 3,
            'mach_optimize': False,
            'mach_initial': (0.5, 'unitless'),
            'mach_final': (0.5, 'unitless'),
            'mach_bounds': ((0.45, 0.55), 'unitless'),
            'mach_polynomial_order': 3,
            'altitude_optimize': False,
            'altitude_initial': (28000.0, 'ft'),
            'altitude_final': (28000.0, 'ft'),
            'altitude_bounds': ((26_000.0, 30_000.0), 'ft'),
            'altitude_polynomial_order': 3,
            'throttle_enforcement': 'boundary_constraint',
            'time_initial_bounds': ((30.0, 180.0), 'min'),
            'time_duration_bounds': ((30.0, 180.0), 'min'),
        },
    },
    'descent': {
        'external_subsystems': [
            CustomAeroBuilder(),
            BatteryBuilder()
            ],
        'subsystem_options': {'core_aerodynamics': {'method': 'external'}},
        'user_options': {
            'num_segments': 5,
            'order': 3,
            'mach_optimize': False,
            'mach_initial': (0.5, 'unitless'),
            'mach_final': (0.3, 'unitless'),
            'mach_bounds': ((0.25, 0.55), 'unitless'),
            'mach_polynomial_order': 3,
            'altitude_optimize': False,
            'altitude_initial': (28000.0, 'ft'),
            'altitude_final': (500.0, 'ft'),
            'altitude_bounds': ((0.0, 30_000.0), 'ft'),
            'altitude_polynomial_order': 3,
            'throttle_enforcement': 'path_constraint',
            'time_initial_bounds': ((60.0, 210.0), 'min'),
            'time_duration_bounds': ((30.0, 90.0), 'min'),
        },
    },
    'post_mission': {
        'include_landing': False,
        'include_batteries': True,
        'constrain_range': True,
        'target_range': (1906.0, 'nmi'),
    },
}

if __name__ == '__main__':
    prob = av.AviaryProblem()
    
    # Load aircraft and options data from user
    prob.load_inputs('aviary/models/aircraft/test_aircraft/aircraft_for_bench_FwFm.csv', phase_info)
    
    # Preprocess inputs
    prob.check_and_preprocess_inputs()

    prob.add_pre_mission_systems()
    prob.add_phases()
    prob.add_post_mission_systems()

    # Link phases and variables
    prob.link_phases()

    # Note, SLSQP has trouble here.
    prob.add_driver('SLSQP')

    prob.add_design_variables() 
    prob.add_objective()

    prob.setup()
    
    prob.model.set_val(Aircraft.CrewPayload.TOTAL_PAYLOAD_MASS, 20_000, units='lbm')
    prob.model.set_val(Aircraft.Design.OPERATING_MASS, 175_400-80_000, units='lbm')
    prob.set_val(av.Aircraft.Battery.PACK_ENERGY_DENSITY, 2360*3.6, units='kJ/kg')
    prob.set_val(av.Aircraft.Battery.PACK_MASS, 28_000, units='lbm')

    prob.set_initial_guesses()

    print("Running model first...")
    prob.run_model()
    
    print("Running check_totals to identify derivative issues...")
    
    # Get the actual design variables to check derivatives with respect to them
    design_vars = list(prob.model.get_design_vars().keys())
    print(f"Design variables: {design_vars}")
    
    # Run check_totals only with respect to design variables
    prob.check_totals(of=design_vars, wrt=design_vars, compact_print=True, show_only_incorrect=True)
    
    print("Check_totals completed. Look for any derivative calculation errors above.")
