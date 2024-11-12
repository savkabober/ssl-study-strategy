"""код состояний игры"""
import bridge.const as const
import bridge.router.waypoint as wp
from bridge.auxiliary import aux, fld
from bridge.strategy.easy_strategy import attacker, goalkeeper


def halt(field: fld.Field, waypoints: list[wp.Waypoint]) -> None:  # TODO: проверить что роботы останавливаются на самом деле
    """Пауза по команде от судей"""
    for i in range(const.TEAM_ROBOTS_MAX_COUNT):
        if field.allies[i].is_used():
            waypoint = wp.Waypoint(
                field.allies[i].get_pos(),
                field.allies[i].get_angle(),
                wp.WType.S_ENDPOINT,
            )
            waypoints[i] = waypoint


def timeout(field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
    """Таймаут по команде от судей"""
    rC = 0
    for i in range(const.TEAM_ROBOTS_MAX_COUNT):
        if field.allies[i].is_used():
            waypoint = wp.Waypoint(
                aux.Point(800 * field.polarity, 300 - 300 * rC),
                0,
                wp.WType.S_ENDPOINT,
            )
            waypoints[i] = waypoint
            rC += 1

