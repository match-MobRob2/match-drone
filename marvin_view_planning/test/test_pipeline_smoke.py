"""Smoke-Test: laeuft die geraycastete Pipeline auf der synthetischen Box-Szene
durch und liefert eine plausible Route? Kein Anspruch auf Optimalitaet -- nur
"stuerzt nicht ab, Ergebnis ist nicht leer".

Benoetigt open3d/scipy/ortools (siehe README.md dieses Pakets); nicht Teil der
Standard-ament-Linter, manuell ausfuehren:

    python3 -m pytest test/test_pipeline_smoke.py
"""
from marvin_view_planning.pipeline import plan_scan_route


def test_plan_scan_route_box_smoke():
    result = plan_scan_route(scene_name="box", resolution=1.0, method="greedy",
                             time_limit=5.0)

    assert len(result.selected) > 0
    assert len(result.route.order) == len(result.selected)
    assert result.tracking is not None
    assert len(result.tracking.stations) > 0
