import numpy as np
import openmdao.api as om

from aviary.subsystems.energy.battery_sizing import SizeBattery, EnergyCapacity, StateofCharge
from aviary.subsystems.subsystem_builder_base import SubsystemBuilderBase
from aviary.variable_info.variables import Aircraft, Dynamic


class BatteryBuilder(SubsystemBuilderBase):
    """
    Builder for the battery model. This simplified battery is sized with a simple energy density relation, and tracks state of charge over the mission (with an efficiency).

    Methods
    -------
    build_pre_mission(self, aviary_inputs=None) -> openmdao.core.System:
        Builds an OpenMDAO system for the pre-mission computations of the subsystem.
    build_mission(self, num_nodes, aviary_inputs=None) -> om.Group:
        Builds an OpenMDAO system for the mission computations of the subsystem.
    get_mass_names(self) -> list:
        Returns the name of variable Aircraft.Battery.MASS as a list
    get_states(self) -> dict:
        Returns a dictionary of the subsystem's states, where the keys are the names
        of the state variables, and the values are dictionaries that contain the units
        for the state variable and any additional keyword arguments required by OpenMDAO
        for the state variable.
    get_constraints(self) -> dict:
        Returns a dictionary of constraints for the battery subsystem.

    """

    default_name = 'battery'

    def build_pre_mission(self, aviary_inputs=None):
        return SizeBattery(aviary_inputs=aviary_inputs)

    def get_mass_names(self):
        return [Aircraft.Battery.MASS]

    def build_mission(self, num_nodes, aviary_inputs=None) -> om.Group:
        battery_group = om.Group()
        # Here, the efficiency variable is used as an overall efficiency for the battery
        soc = om.ExecComp(
            'state_of_charge = (energy_capacity - (cumulative_electric_energy_used/efficiency)) / energy_capacity',
            state_of_charge={'val': np.zeros(num_nodes), 'units': 'unitless'},
            energy_capacity={'val': 10.0, 'units': 'kJ'},
            cumulative_electric_energy_used={'val': np.zeros(num_nodes), 'units': 'kJ'},
            efficiency={'val': 0.95, 'units': 'unitless'},
            has_diag_partials=True,
        )

        battery_group.add_subsystem(
            'state_of_charge',
            subsys=soc,
            promotes_inputs=[
                ('energy_capacity', Aircraft.Battery.ENERGY_CAPACITY),
                (
                    'cumulative_electric_energy_used',
                    Dynamic.Vehicle.CUMULATIVE_ELECTRIC_ENERGY_USED,
                ),
                ('efficiency', Aircraft.Battery.EFFICIENCY),
            ],
            promotes_outputs=[('state_of_charge', Dynamic.Vehicle.BATTERY_STATE_OF_CHARGE)],
        )
        
        # energy_capacity = om.ExecComp(
        #     'energy_capacity = pack_mass * pack_energy_density',
        #     energy_capacity={'val': 0.0, 'units': 'kJ'},
        #     pack_mass={'val': 3_000.0, 'units': 'kg'},
        #     pack_energy_density={'val': 320.0*3.6, 'units': 'kJ/kg'},
        #     has_diag_partials=True,
        # )

        # battery_group.add_subsystem(
        #     'energy_capacity',
        #     subsys=EnergyCapacity(aviary_inputs=aviary_inputs),
        #     promotes_inputs=[Aircraft.Battery.MASS, Aircraft.Battery.PACK_ENERGY_DENSITY],
        #     promotes_outputs=[Aircraft.Battery.ENERGY_CAPACITY],
        # )
        
        # battery_group.add_subsystem(
        #     'state_of_charge',
        #     subsys=StateofCharge(aviary_inputs=aviary_inputs, num_nodes=num_nodes),
        #     promotes_inputs=[Aircraft.Battery.ENERGY_CAPACITY,
        #                      Dynamic.Vehicle.CUMULATIVE_ELECTRIC_ENERGY_USED,
        #                      Aircraft.Battery.EFFICIENCY,
        #     ],
        #     promotes_outputs=[Dynamic.Vehicle.BATTERY_STATE_OF_CHARGE],
        # )

        return battery_group

    def get_states(self):
        # need to add subsystem name to target name ('battery.') for state due
        # to issue where non aircraft or mission variables are not fully promoted
        # TODO fix this by not promoting only 'aircraft:*' and 'mission:*'
        state_dict = {
            Dynamic.Vehicle.CUMULATIVE_ELECTRIC_ENERGY_USED: {
                'fix_initial': True,
                'fix_final': False,
                'lower': 0.0,
                'ref': 1e4,
                'defect_ref': 1e6,
                'units': 'kJ',
                'rate_source': Dynamic.Vehicle.Propulsion.ELECTRIC_POWER_IN_TOTAL,
                'input_initial': 0.0,
                'targets': f'{self.name}.{Dynamic.Vehicle.CUMULATIVE_ELECTRIC_ENERGY_USED}',
            }
        }

        return state_dict

    def get_constraints(self):
        constraint_dict = {
            # Can add constraints here; state of charge is a common one in many
            # battery applications
            f'{self.name}.{Dynamic.Vehicle.BATTERY_STATE_OF_CHARGE}': {
                'type': 'boundary',
                'loc': 'final',
                'lower': 0.1,
            },
        }
        return constraint_dict
    
    def get_design_vars(self):
        """
        Return a dictionary of design variables for the subsystem.

        Optional

        Not currently used.

        Returns
        -------
        design_vars : dict
            A dictionary where the keys are the names of the design variables
            and the values are dictionaries with the following keys:

            - any additional keyword arguments required by OpenMDAO for the design
              variable.
        """
        pack_dict = {
            'units': 'kg',
            'lower': 0,
            'ref': 1,
        }  # upper and lower are just notional for now
        design_vars = {
            Aircraft.Battery.PACK_MASS: pack_dict,
        }
        return design_vars

    def get_parameters(self, aviary_inputs=None, phase_info=None):
        params = {
            # Aircraft.Battery.ENERGY_CAPACITY: {
            #     'val': 30_000.0,
            #     'units': 'kJ',
            # },
        }
        return params

    def get_linked_variables(self):
        # link cumulative electric energy between phases
        return [Dynamic.Vehicle.CUMULATIVE_ELECTRIC_ENERGY_USED]
