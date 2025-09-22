from __future__ import annotations
import numpy as np
from typing import Tuple, Iterable
import aviary.api as av
import openmdao.api as om


def build_aero_tables(
    altitude: Iterable[float],
    mach: Iterable[float],
    angle_of_attack: Iterable[float],
    aspect_ratio: float,
    oswald_factor: float,
    cd0: float,
    *,
    aoa_in_degrees: bool = False,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Build 3D CL and CD tables on (altitude, mach, angle_of_attack) grids.

    Assumptions:
      - Simplified lift: CL = 2*pi*alpha (linear, small-angle); no dependence on M or altitude.
      - Drag: CD = CD0 + CL^2/(pi*AR*e); no dependence on M or altitude.

    Args:
        altitude: 1D grid of altitudes.
        mach: 1D grid of Mach numbers.
        angle_of_attack: 1D grid of angle of attack (radians by default).
        aspect_ratio: Wing aspect ratio (AR).
        oswald_factor: Oswald efficiency factor (e).
        cd0: Zero-lift drag coefficient (CD0).
        aoa_in_degrees: If True, convert AoA from degrees to radians.

    Returns:
        (CL_table, CD_table) each with shape (n_alt, n_mach, n_aoa).
    """
    alt = np.asarray(altitude, dtype=float)
    mch = np.asarray(mach, dtype=float)
    aoa = np.asarray(angle_of_attack, dtype=float)

    if aoa_in_degrees:
        aoa = np.deg2rad(aoa)

    n_alt = alt.size
    n_mach = mch.size
    n_aoa = aoa.size

    # Lift slope: 2*pi per rad
    cl_1d = 2.0 * np.pi * aoa  # shape (n_aoa,)

    # Broadcast to 3D: (n_alt, n_mach, n_aoa)
    cl_table = np.broadcast_to(cl_1d[None, None, :], (n_alt, n_mach, n_aoa))

    # Parasitic + induced drag
    cd_1d = cd0 + (cl_1d ** 2) / (np.pi * aspect_ratio * oswald_factor)
    cd_table = np.broadcast_to(cd_1d[None, None, :], (n_alt, n_mach, n_aoa))

    return cl_table, cd_table

class AeroTableBuilder(om.ExplicitComponent):
    """
    OpenMDAO component that builds CL and CD tables over (altitude, mach, aoa).
    """

    def initialize(self):
        self.options.declare("altitude", types=(list, tuple, np.ndarray),
                             desc="Altitude grid [any units], 1D")
        self.options.declare("mach", types=(list, tuple, np.ndarray),
                             desc="Mach grid, 1D")
        self.options.declare("angle_of_attack", types=(list, tuple, np.ndarray),
                             desc="AoA grid [rad by default], 1D")
        self.options.declare("aspect_ratio", types=float, desc="Wing aspect ratio (AR)")
        self.options.declare("oswald_factor", types=float, desc="Oswald efficiency factor (e)")
        self.options.declare("cd0", types=float, desc="Zero-lift drag coefficient (CD0)")
        self.options.declare("aoa_in_degrees", types=bool, default=False,
                             desc="Interpret AoA grid as degrees if True")

    def setup(self):
        alt = np.atleast_1d(np.asarray(self.options["altitude"], dtype=float))
        mch = np.atleast_1d(np.asarray(self.options["mach"], dtype=float))
        aoa = np.atleast_1d(np.asarray(self.options["angle_of_attack"], dtype=float))

        n_alt, n_mach, n_aoa = alt.size, mch.size, aoa.size

        self.add_output("CL_table", shape=(n_alt, n_mach, n_aoa))
        self.add_output("CD_table", shape=(n_alt, n_mach, n_aoa))

        # No inputs yet; tables are purely option-driven for now.

    def compute(self, inputs, outputs):
        alt = self.options["altitude"]
        mch = self.options["mach"]
        aoa = self.options["angle_of_attack"]
        ar = self.options["aspect_ratio"]
        e = self.options["oswald_factor"]
        cd0 = self.options["cd0"]
        aoa_deg = self.options["aoa_in_degrees"]

        cl_tbl, cd_tbl = build_aero_tables(
            altitude=alt,
            mach=mch,
            angle_of_attack=aoa,
            aspect_ratio=ar,
            oswald_factor=e,
            cd0=cd0,
            aoa_in_degrees=aoa_deg,
        )

        outputs["CL_table"] = cl_tbl
        outputs["CD_table"] = cd_tbl
        
class ExternalAeroBuilder(av.AerodynamicsBuilderBase):
    """
    An example subsystem builder that adds an external aerodynamics component.

    Parameters
    ----------
    aero_data : NamedValues
        Altitude, Mach number, and angle of attack data, all in ascending order.
    """

    def __init__(self, name='aero', altitude=None, mach=None, angle_of_attack=None,
                 aspect_ratio=None, oswald_factor=None, cd0=None):
        super().__init__(name)
        self.altitude = altitude
        self.mach = mach
        self.angle_of_attack = angle_of_attack
        self.aspect_ratio=aspect_ratio
        self.oswald_factor=oswald_factor
        self.cd0=cd0

    def build_pre_mission(self, aviary_inputs):
        """
        Build an OpenMDAO system for the pre-mission computations of the subsystem.

        Returns
        -------
        pre_mission_sys : openmdao.core.Group
            An OpenMDAO group containing all computations that need to happen in
            the pre-mission part of the Aviary problem. This includes sizing, design,
            and other non-mission parameters.
        """
        aero_group = om.Group()
        aero = AeroTableBuilder(
            altitude=self.altitude, mach=self.mach, angle_of_attack=self.angle_of_attack,
            aspect_ratio=self.aspect_ratio, oswald_factor=self.oswald_factor, cd0=self.cd0
        )
        aero_group.add_subsystem(
            'premission_aero',
            aero,
            promotes_inputs=['*'],
            promotes_outputs=[
                ('drag_table', av.Aircraft.Design.DRAG_POLAR),
                ('lift_table', av.Aircraft.Design.LIFT_POLAR),
            ],
        )
        return aero_group