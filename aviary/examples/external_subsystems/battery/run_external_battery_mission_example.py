"""Example mission using the a detailed battery model."""

from aviary.api import default_height_energy_phase_info as phase_info
from aviary.examples.external_subsystems.battery.battery_builder import BatteryBuilder
from aviary.examples.external_subsystems.battery.battery_variable_meta_data import ExtendedMetaData
from aviary.interface.methods_for_level2 import AviaryProblem
from aviary.utils.functions import get_aviary_resource_path
from aviary.api import Aircraft, Mission, Settings, ProblemType
from aviary.variable_info.variables import Dynamic
import aviary.api as av


battery_builder = BatteryBuilder(include_constraints=False)

# add the battery model to each mission phase, as well as pre-mission for sizing
phase_info['pre_mission']['external_subsystems'] = [battery_builder]
phase_info['climb']['external_subsystems'] = [battery_builder]
phase_info['cruise']['external_subsystems'] = [battery_builder]
phase_info['descent']['external_subsystems'] = [battery_builder]

if __name__ == '__main__':
    prob = AviaryProblem()

    # Load aircraft and options data from user
    # Allow for user overrides here
    input_file = get_aviary_resource_path(
        'aviary/models/aircraft/test_aircraft/aircraft_for_bench_FwFm_with_electric.csv'
    )
    prob.load_inputs(input_file, phase_info, meta_data=ExtendedMetaData)

    # Preprocess inputs
    prob.check_and_preprocess_inputs()

    prob.add_pre_mission_systems()

    prob.add_phases()

    prob.add_post_mission_systems()

    # Link phases and variables
    prob.link_phases()

    prob.add_driver('SLSQP')

    prob.add_design_variables()

    prob.add_objective('mass')

    prob.setup()

    prob.set_initial_guesses()
    
    prob.model.set_val(Aircraft.Design.OPERATING_MASS, 80_000, units='lbm')
    prob.model.set_val(Aircraft.CrewPayload.TOTAL_PAYLOAD_MASS, 20_000, units='lbm')
    # prob.model.set_val(Aircraft.Battery.PACK_ENERGY_DENSITY, 4360*3.6, units='kJ/kg')
    # prob.model.set_val(Aircraft.Battery.PACK_MASS, 28_000, units='lbm')

    prob.run_aviary_problem()

    # import numpy as np
    # print(np.round(prob[Mission.Summary.GROSS_MASS], 0))
    # print(np.round(prob[Mission.Summary.TOTAL_FUEL_MASS], 0))
    # print(np.round(prob[Mission.Summary.FUEL_BURNED], 0))
    # print(np.round(prob[Mission.Summary.RANGE], 0))

    # print(f"RANGE: {prob[Mission.Summary.RANGE][0]:.0f} km")
    # print(f"TOTAL_PAYLOAD_MASS: {prob[Aircraft.CrewPayload.TOTAL_PAYLOAD_MASS][0]:.0f} lbm")

    # print("Running model first...")
    
    # print("Running check_totals to identify derivative issues...")
    
    # # Run check_totals to identify derivative problems
    # prob.check_totals(compact_print=True, show_only_incorrect=True)
    
    # print('done')
    # print(prob)