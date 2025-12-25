"""CasADi + Pinocchio IK solver for Mantis arms.

This module is the **minimal** solver surface needed by `mantis_casadi_node`.
It was extracted during repo cleanup so phase1 can still run with `enable_ik:=true`
without keeping unrelated legacy/debug files.

Contract:
- `MantisArmIK().get_initial_fk() -> (T_left, T_right)`
- `MantisArmIK().solve_ik(T_left, T_right) -> q` (numpy array)
- `MantisArmIK().get_joint_names() -> list[str]` arm joint names, in the same
  order as `q`.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import casadi
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin
from ament_index_python.packages import get_package_share_directory


LEFT_ARM_JOINTS = [
    "L_Shoulder_Pitch_Joint",
    "L_Shoulder_Yaw_Joint",
    "L_Shoulder_Roll_Joint",
    "L_Elbow_Pitch_Joint",
    "L_Wrist_Roll_Joint",
    "L_Wrist_Pitch_Joint",
    "L_Wrist_Yaw_Joint",
]

RIGHT_ARM_JOINTS = [
    "R_Shoulder_Pitch_Joint",
    "R_Shoulder_Yaw_Joint",
    "R_Shoulder_Roll_Joint",
    "R_Elbow_Pitch_Joint",
    "R_Wrist_Roll_Joint",
    "R_Wrist_Pitch_Joint",
    "R_Wrist_Yaw_Joint",
]


@dataclass
class _ReducedRobot:
    model: pin.Model
    data: pin.Data


class MantisArmIK:
    """A small damped-least-squares IK wrapper."""

    def __init__(self):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        pkg_dir = get_package_share_directory('mantis_description')
        urdf_path = os.path.join(pkg_dir, 'urdf', 'mantis.urdf')

        robot = pin.RobotWrapper.BuildFromURDF(urdf_path, pkg_dir)
        full_model = robot.model

        # Build reduced model: lock all joints except arm joints.
        whitelist = set(LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS)
        joints_to_lock: list[int] = []
        for name in full_model.names:
            if name == 'universe':
                continue
            if name not in whitelist:
                joints_to_lock.append(full_model.getJointId(name))
        joints_to_lock = sorted(set(joints_to_lock))

        reduced_robot = robot.buildReducedRobot(
            list_of_joints_to_lock=joints_to_lock,
            reference_configuration=pin.neutral(full_model),
        )
        self._robot = _ReducedRobot(model=reduced_robot.model, data=reduced_robot.data)

        # End-effector frames (using wrist yaw joint frame names from earlier code).
        self._left_ee_frame = 'L_Wrist_Yaw_Joint'
        self._right_ee_frame = 'R_Wrist_Yaw_Joint'
        self._l_id = self._robot.model.getFrameId(self._left_ee_frame)
        self._r_id = self._robot.model.getFrameId(self._right_ee_frame)

        self._q = pin.neutral(self._robot.model)
        pin.framesForwardKinematics(self._robot.model, self._robot.data, self._q)

        self._joint_names = [
            n for n in list(self._robot.model.names)
            if n != 'universe'
        ]

        self._setup_casadi()

    def get_joint_names(self) -> list[str]:
        return list(self._joint_names)

    def get_initial_fk(self):
        pin.framesForwardKinematics(self._robot.model, self._robot.data, self._q)
        T_l = self._robot.data.oMf[self._l_id].homogeneous
        T_r = self._robot.data.oMf[self._r_id].homogeneous
        return T_l, T_r

    def _setup_casadi(self):
        self._cmodel = cpin.Model(self._robot.model)
        self._cdata = self._cmodel.createData()

        self._cq = casadi.SX.sym('q', self._robot.model.nq, 1)
        self._cTf_l = casadi.SX.sym('tf_l', 4, 4)
        self._cTf_r = casadi.SX.sym('tf_r', 4, 4)

        cpin.framesForwardKinematics(self._cmodel, self._cdata, self._cq)

        pos_err_l = self._cdata.oMf[self._l_id].translation - self._cTf_l[:3, 3]
        pos_err_r = self._cdata.oMf[self._r_id].translation - self._cTf_r[:3, 3]
        rot_err_l = cpin.log3(self._cdata.oMf[self._l_id].rotation @ self._cTf_l[:3, :3].T)
        rot_err_r = cpin.log3(self._cdata.oMf[self._r_id].rotation @ self._cTf_r[:3, :3].T)

        self._error_func = casadi.Function(
            'error_func',
            [self._cq, self._cTf_l, self._cTf_r],
            [casadi.vertcat(pos_err_l, pos_err_r, rot_err_l, rot_err_r)],
        )

        self._opti = casadi.Opti()
        self._var_q = self._opti.variable(self._robot.model.nq)
        self._var_q_last = self._opti.parameter(self._robot.model.nq)
        self._param_tf_l = self._opti.parameter(4, 4)
        self._param_tf_r = self._opti.parameter(4, 4)

        errors = self._error_func(self._var_q, self._param_tf_l, self._param_tf_r)
        cost = (
            50.0 * casadi.sumsqr(errors[:6])
            + 10.0 * casadi.sumsqr(errors[6:])
            + 0.01 * casadi.sumsqr(self._var_q)
            + 1.0 * casadi.sumsqr(self._var_q - self._var_q_last)
        )

        self._opti.minimize(cost)
        self._opti.subject_to(
            self._opti.bounded(
                self._robot.model.lowerPositionLimit,
                self._var_q,
                self._robot.model.upperPositionLimit,
            )
        )

        opts = {'ipopt': {'print_level': 0, 'max_iter': 30, 'tol': 1e-4}, 'print_time': False}
        self._opti.solver('ipopt', opts)

    def solve_ik(self, T_left: np.ndarray, T_right: np.ndarray) -> np.ndarray:
        self._opti.set_initial(self._var_q, self._q)
        self._opti.set_value(self._var_q_last, self._q)
        self._opti.set_value(self._param_tf_l, T_left)
        self._opti.set_value(self._param_tf_r, T_right)

        try:
            sol = self._opti.solve()
            q_sol = sol.value(self._var_q)
        except Exception:
            q_sol = self._opti.debug.value(self._var_q)

        q_sol = np.array(q_sol).reshape((-1,))
        self._q = q_sol
        return q_sol
