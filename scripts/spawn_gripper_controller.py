#!/usr/bin/env python3
"""
One-shot script: loads and activates gripper_position_controller
(JointGroupPositionController for robotiq_85_left_knuckle_joint).

GripperActionController absent from Jazzy → use JointGroupPositionController.
Commands are then published as Float64MultiArray to
/gripper_position_controller/commands.
"""

import rclpy
import rclpy.duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from controller_manager_msgs.srv import (
    LoadController, ConfigureController, SwitchController,
)

CONTROLLER_NAME  = "gripper_position_controller"
CONTROLLER_TYPE  = "position_controllers/JointGroupPositionController"
CONTROLLER_JOINT = "robotiq_85_left_knuckle_joint"

# Couleur du node dans le terminal (gripper spawner = bleu)
_BL = "\033[1;34m"
_RST = "\033[0m"


def wait_call(executor, node, future, timeout=10.0):
    """Spin the executor until the future is done."""
    executor.spin_until_future_complete(future, timeout_sec=timeout)
    return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = Node("gripper_controller_spawner")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    log = node.get_logger()

    try:
        # ── 1. Set controller type on /controller_manager ─────────────────
        set_params = node.create_client(
            SetParameters, "/controller_manager/set_parameters"
        )
        log.info("Waiting for /controller_manager/set_parameters …")
        while not set_params.wait_for_service(timeout_sec=2.0):
            if not rclpy.ok():
                return
            log.info("  … still waiting")

        p_type = Parameter(
            name=f"{CONTROLLER_NAME}.type",
            value=ParameterValue(
                type=ParameterType.PARAMETER_STRING,
                string_value=CONTROLLER_TYPE,
            ),
        )
        req = SetParameters.Request()
        req.parameters = [p_type]
        result = wait_call(executor, node, set_params.call_async(req))
        log.info(f"Type set → {CONTROLLER_TYPE}")

        # ── 2. Set controller joints / interface_name ──────────────────────
        p_joints = Parameter(
            name=f"{CONTROLLER_NAME}.joints",
            value=ParameterValue(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                string_array_value=[CONTROLLER_JOINT],
            ),
        )
        p_iface = Parameter(
            name=f"{CONTROLLER_NAME}.interface_name",
            value=ParameterValue(
                type=ParameterType.PARAMETER_STRING,
                string_value="position",
            ),
        )
        req2 = SetParameters.Request()
        req2.parameters = [p_joints, p_iface]
        wait_call(executor, node, set_params.call_async(req2))
        log.info("Joint params set.")

        # ── 3. Load controller ─────────────────────────────────────────────
        load = node.create_client(LoadController, "/controller_manager/load_controller")
        load.wait_for_service()
        req3 = LoadController.Request()
        req3.name = CONTROLLER_NAME
        res3 = wait_call(executor, node, load.call_async(req3))
        if not (res3 and res3.ok):
            log.error(f"Load failed: {res3}")
            return
        log.info("Controller loaded.")

        # ── 4a. Set params on the controller's own node (created after load) ──
        ctrl_params = node.create_client(
            SetParameters, f"/{CONTROLLER_NAME}/set_parameters"
        )
        log.info(f"Waiting for /{CONTROLLER_NAME}/set_parameters …")
        for _ in range(10):
            if ctrl_params.wait_for_service(timeout_sec=1.0):
                break
            executor.spin_once(timeout_sec=0.1)

        if ctrl_params.service_is_ready():
            p_j = Parameter(
                name="joints",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING_ARRAY,
                    string_array_value=[CONTROLLER_JOINT],
                ),
            )
            p_i = Parameter(
                name="interface_name",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING,
                    string_value="position",
                ),
            )
            req_ctrl = SetParameters.Request()
            req_ctrl.parameters = [p_j, p_i]
            wait_call(executor, node, ctrl_params.call_async(req_ctrl))
            log.info("Controller-node params set.")

        # ── 4b. Configure controller ───────────────────────────────────────
        conf = node.create_client(
            ConfigureController, "/controller_manager/configure_controller"
        )
        conf.wait_for_service()
        req4 = ConfigureController.Request()
        req4.name = CONTROLLER_NAME
        res4 = wait_call(executor, node, conf.call_async(req4))
        if not (res4 and res4.ok):
            log.error(f"Configure failed: {res4}")
            return
        log.info("Controller configured.")

        # ── 5. Activate controller ─────────────────────────────────────────
        switch = node.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        switch.wait_for_service()
        req5 = SwitchController.Request()
        req5.activate_controllers   = [CONTROLLER_NAME]
        req5.deactivate_controllers = []
        req5.strictness             = SwitchController.Request.BEST_EFFORT
        req5.activate_asap          = True
        req5.timeout                = rclpy.duration.Duration(seconds=5.0).to_msg()
        res5 = wait_call(executor, node, switch.call_async(req5))
        if not (res5 and res5.ok):
            log.error(f"Activate failed: {res5}")
            return
        log.info(f"{_BL}✓ {CONTROLLER_NAME} activé — pince prête.{_RST}")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
