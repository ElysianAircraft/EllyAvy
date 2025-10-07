"""Example mission using the a detailed battery model."""

from aviary.api import default_height_energy_phase_info as phase_info
# from aviary.examples.external_subsystems.battery.battery_builder import BatteryBuilder
from aviary.subsystems.energy.battery_builder import BatteryBuilder
from aviary.examples.external_subsystems.battery.battery_variable_meta_data import ExtendedMetaData
from aviary.interface.methods_for_level2 import AviaryProblem
from aviary.utils.functions import get_aviary_resource_path
from aviary.variable_info.variables import Aircraft, Mission
from aviary.examples.external_subsystems.battery.battery_variables import Dynamic

battery_builder = BatteryBuilder()

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
        'models/aircraft/test_aircraft/aircraft_for_bench_FwFm_with_electric.csv'
    )
    prob.load_inputs(input_file, phase_info, meta_data=ExtendedMetaData)

    prob.check_and_preprocess_inputs()

    prob.build_model()

    prob.add_driver('SLSQP')

    prob.add_design_variables()

    prob.add_objective()
    # prob.model.add_objective(
    #     f'traj.climb.states:{Dynamic.Battery.STATE_OF_CHARGE}', index=-1, ref=-1)

    prob.setup()

    prob.run_aviary_problem()
    import numpy as np
    # print(np.round(prob.get_val('aircraft:battery:energy_required'), 2))
    print(np.round(prob.get_val(Aircraft.Battery.MASS), 2))
    print(np.round(prob.get_val(Mission.Summary.GROSS_MASS), 2))
    print(np.round(prob.get_val(Aircraft.Design.OPERATING_MASS), 2))
    print(np.round(prob.get_val(Aircraft.CrewPayload.TOTAL_PAYLOAD_MASS), 2))
    print(np.round(prob.get_val('traj.descent.timeseries.cumulative_electric_energy_used'), 2))
