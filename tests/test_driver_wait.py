"""Asynchronous commands are tracked by RTDE status, independent of velocity."""

from unittest.mock import Mock, patch

import pytest
from rtde_control import AsyncOperationStatus

from ur_arm_client import ConnectionError, OperationError, URArmClient

TARGET = [0.0] * 6


def status(operation_id, running=False):
    return AsyncOperationStatus((operation_id << 24) | (0x8000 if running else 0))


@pytest.fixture
def client_and_controller():
    controller = Mock()
    controller.moveJ.return_value = True
    controller.moveL.return_value = True
    with (
        patch("ur_arm_client._client.RTDEController", return_value=controller),
        patch("ur_arm_client._client.RTDEReceiver"),
        URArmClient("robot") as client,
    ):
        yield client, controller


def test_wait_for_start_then_finish(client_and_controller):
    client, controller = client_and_controller
    progress = controller.getAsyncOperationProgressEx
    progress.side_effect = [status(5), status(5), status(6, True), status(6)]

    token = client.movej(TARGET, blocking=False)
    client.wait_done(token)

    assert progress.call_count == 4
    client.wait_done(token, timeout=0)
    assert progress.call_count == 4


def test_fast_move_can_finish_before_first_poll_and_wrap_counter(client_and_controller):
    client, controller = client_and_controller
    controller.getAsyncOperationProgressEx.side_effect = [status(127), status(0)]
    client.movel(TARGET, blocking=False)

    client.wait_all_done(timeout=0)


def test_timeout_does_not_discard_pending_move(client_and_controller):
    client, controller = client_and_controller
    controller.getAsyncOperationProgressEx.side_effect = [
        status(10),
        status(10),
        status(11),
    ]
    token = client.movej(TARGET, blocking=False)

    with pytest.raises(TimeoutError, match="did not complete"):
        client.wait_done(token, timeout=0)

    client.wait_done(token, timeout=0)


def test_new_move_cannot_replace_pending_move(client_and_controller):
    client, controller = client_and_controller
    controller.getAsyncOperationProgressEx.side_effect = [
        status(1),
        status(2, True),
        status(2),
    ]
    token = client.movej(TARGET, blocking=False)

    with pytest.raises(OperationError, match="already pending"):
        client.movel(TARGET)

    controller.moveL.assert_not_called()
    client.wait_done(token)


def test_different_controller_operation_does_not_complete_token(client_and_controller):
    client, controller = client_and_controller
    controller.getAsyncOperationProgressEx.side_effect = [status(10), status(12)]

    token = client.movej(TARGET, blocking=False)

    with pytest.raises(OperationError, match="different operation"):
        client.wait_done(token)


def test_stopping_motion_invalidates_its_token(client_and_controller):
    client, controller = client_and_controller
    controller.getAsyncOperationProgressEx.return_value = status(7)
    token = client.movej(TARGET, blocking=False)

    client.stopj()

    with pytest.raises(OperationError, match="interrupted"):
        client.wait_done(token)


def test_tokens_do_not_cross_sessions(client_and_controller):
    client, controller = client_and_controller
    controller.getAsyncOperationProgressEx.return_value = status(3)
    token = client.movej(TARGET, blocking=False)

    with URArmClient("other robot") as other:
        with pytest.raises(OperationError, match="does not belong"):
            other.wait_done(token)

    client.disconnect()
    client.connect()
    with pytest.raises(OperationError, match="does not belong"):
        client.wait_done(token)


def test_wait_all_done_is_noop_without_pending_move(client_and_controller):
    client, controller = client_and_controller
    client.wait_all_done()
    controller.getAsyncOperationProgressEx.assert_not_called()


def test_wait_requires_connection():
    client = URArmClient("robot")
    with pytest.raises(ConnectionError, match="Not connected"):
        client.wait_done(None)
    with pytest.raises(ConnectionError, match="Not connected"):
        client.wait_all_done()
