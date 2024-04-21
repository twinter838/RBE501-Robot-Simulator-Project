import importlib.util
import os.path
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np
import numpy.testing as np_test
import pinocchio as pin

from timor import Robot
from timor.Module import ModuleAssembly, ModulesDB
from timor import Geometry
from timor.utilities import logging


def allclose(x: np.ndarray, y: np.ndarray) -> bool:
    """Uses np.allclose with custom tolerance. 1e-6 is max. allowed abs. tolerance, no relative tolerance"""
    return np.allclose(x, y, atol=1e-6, rtol=0)


def robot_io_equal(m1: Robot.PinRobot, m2: Robot.PinRobot, n_iter: int = 1000) -> bool:
    """
    Make sure two robots behave the same way in kinematics and dynamics
    """
    for _ in range(n_iter):
        q, dq, ddq = m1.random_configuration(), m1.random_configuration(), m1.random_configuration()
        wrench = np.random.random((6,))
        if not all((allclose(m1.id(q, dq, ddq), m2.id(q, dq, ddq)),
                    allclose(m1.id(q, dq, ddq, eef_wrench=wrench), m2.id(q, dq, ddq, eef_wrench=wrench)),
                    m1.fk(q) == m2.fk(q),
                    allclose(m1.fd(ddq, q, dq), m2.fd(ddq, q, dq)))):
            logging.warn(f"Discrepancy for q = {q}, dq = {dq}, ddq / tau = {ddq}, wrench = {wrench}")
            return False
    return True


def pin_geometry_models_functionally_equal(r1: Robot.RobotBase, r2: Robot.RobotBase,
                                           iterations: int = 100) -> float:
    """Tests if (collision) geometry models of two robots are functionally equal - find the same self-collisions."""
    difference = 0
    for i in range(iterations):
        q = r1.random_configuration()
        if r1.has_self_collision(q) != r2.has_self_collision(q):
            logging.warn(f"Self-collision differ for {q}")
    return difference / iterations


class TestURDF2JSON(unittest.TestCase):
    """Tests properties and methods of the Module implementation."""

    def setUp(self) -> None:
        self.package_dir = Path(__file__).parent.parent
        self.module_path = self.package_dir.joinpath("modules.json")
        self.panda_urdf = self.package_dir.joinpath("panda.urdf")

    def test_urdf2json(self):
        self.assertTrue(self.module_path.is_file())
        original_db = ModulesDB.from_file(self.module_path, self.package_dir)

        t_old = os.path.getmtime(self.module_path)

        def run_main(file: Path):
            """
            Imports a single python file and runs its main.
            Taken from: https://stackoverflow.com/questions/67631/how-to-import-a-module-given-the-full-path
            """
            spec = importlib.util.spec_from_file_location("imported", file)
            script = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(script)
            script.main()

        with patch('builtins.input', return_value='y'):  # Default all input calls to 'y'
            run_main(Path(__file__).parent.parent.joinpath("urdf_to_json.py"))
        new_db = ModulesDB.from_file(self.module_path, self.package_dir)

        self.assertNotEqual(t_old, os.path.getmtime(self.module_path), "Did not generate new json...")

        # TODO - Move to DB / Module / Joints / Bodies / Connectors / ... __equ__
        self.assertSetEqual(original_db.all_module_ids, new_db.all_module_ids)
        for id in original_db.all_module_ids:
            m_old, m_new = original_db.by_id[id], new_db.by_id[id]
            self.assertEqual(m_old.header, m_new.header)
            self.assertAlmostEqual(m_old.mass, m_new.mass)

            for b_old in m_old.bodies:
                b_new = next(b for b in m_new.bodies if b.id == b_old.id)
                self.assertAlmostEqual(b_old.inertia.mass, b_new.inertia.mass, msg=f"Mass error for {b_new}")
                np_test.assert_allclose(b_old.inertia.lever, b_new.inertia.lever, err_msg="Lever error for {b_new}")
                np_test.assert_allclose(b_old.inertia.inertia, b_new.inertia.inertia,
                                        err_msg="Inertia error for {b_new}")
                if isinstance(b_old.visual, Geometry.Mesh):
                    self.assertEqual(b_old.visual.abs_filepath, b_new.visual.abs_filepath)

                for c_old in b_old.connectors:
                    c_new = next(c for c in b_new.connectors if c.id == c_old.id)
                    self.assertEqual(c_new.id, c_old.id)
                    self.assertEqual(c_new.body2connector, c_old.body2connector)
                    self.assertEqual(c_new.gender, c_old.gender)
                    self.assertEqual(c_new.type, c_old.type)
                    self.assertSetEqual(set(c_new.size), set(c_old.size))

            for j_old in m_old.joints:
                j_new = next(j for j in m_new.joints if j.id == j_old.id)
                self.assertEqual(j_new.child2joint, j_old.child2joint)
                self.assertEqual(j_new.friction_coulomb, j_old.friction_coulomb)
                self.assertEqual(j_new.friction_viscous, j_old.friction_viscous)
                self.assertEqual(j_new.gear_ratio, j_old.gear_ratio)
                self.assertEqual(j_new.id, j_old.id)
                self.assertEqual(j_new.joint2parent, j_old.joint2parent)
                np_test.assert_allclose(j_new.limits, j_old.limits)
                self.assertEqual(j_new.motor_inertia, j_old.motor_inertia)
                self.assertEqual(j_new.torque_limit, j_old.torque_limit)
                self.assertEqual(j_new.type, j_old.type)
                self.assertEqual(j_new.velocity_limit, j_old.velocity_limit)

    def test_compareURDF_ModuleDB(self):
        r_urdf = Robot.PinRobot.from_urdf(self.panda_urdf, self.package_dir)
        assembly_db = \
            ModuleAssembly.from_serial_modules(ModulesDB.from_file(self.module_path, self.package_dir), ["panda"])
        robot_io_equal(r_urdf, assembly_db.robot)
        self.assertLess(pin_geometry_models_functionally_equal(r_urdf, assembly_db.robot), 0.01)


if __name__ == '__main__':
    unittest.main()
